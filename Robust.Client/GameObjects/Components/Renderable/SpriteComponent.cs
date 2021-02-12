using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.Linq;
using System.Text;
using Robust.Client.Graphics;
using Robust.Client.ResourceManagement;
using Robust.Client.Utility;
using Robust.Shared.Animations;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Log;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.Shared.Network;
using Robust.Shared.Prototypes;
using Robust.Shared.Reflection;
using Robust.Shared.Serialization;
using Robust.Shared.Timing;
using Robust.Shared.Utility;
using Robust.Shared.ViewVariables;
using DrawDepthTag = Robust.Shared.GameObjects.DrawDepth;

namespace Robust.Client.GameObjects
{
    public sealed class SpriteComponent : SharedSpriteComponent, ISpriteComponent,
        IComponentDebug
    {
        private bool _visible = true;

        [ViewVariables(VVAccess.ReadWrite)]
        public bool Visible
        {
            get => _visible;
            set => _visible = value;
        }

        private int drawDepth = DrawDepthTag.Default;

        /// <summary>
        ///     Z-index for drawing.
        /// </summary>
        [ViewVariables(VVAccess.ReadWrite)]
        public int DrawDepth
        {
            get => drawDepth;
            set => drawDepth = value;
        }

        private Vector2 scale = Vector2.One;

        /// <summary>
        ///     A scale applied to all layers.
        /// </summary>
        [Animatable]
        [ViewVariables(VVAccess.ReadWrite)]
        public Vector2 Scale
        {
            get => scale;
            set => scale = value;
        }

        private Angle rotation;

        [Animatable]
        [ViewVariables(VVAccess.ReadWrite)]
        public Angle Rotation
        {
            get => rotation;
            set => rotation = value;
        }

        private Vector2 offset = Vector2.Zero;

        /// <summary>
        ///     Offset applied to all layers.
        /// </summary>
        [Animatable]
        [ViewVariables(VVAccess.ReadWrite)]
        public Vector2 Offset
        {
            get => offset;
            set => offset = value;
        }

        private Color color = Color.White;

        [Animatable]
        [ViewVariables(VVAccess.ReadWrite)]
        public Color Color
        {
            get => color;
            set => color = value;
        }

        /// <summary>
        ///     Controls whether we use RSI directions to rotate, or just get angular rotation applied.
        ///     If true, all rotation to this sprite component is negated (that is rotation from say the owner being rotated).
        ///     Rotation transformations on individual layers still apply.
        ///     If false, all layers get locked to south and rotation is a transformation.
        /// </summary>
        [ViewVariables(VVAccess.ReadWrite)]
        public bool Directional
        {
            get => _directional;
            set => _directional = value;
        }

        private bool _directional = true;

        private RSI? _baseRsi;

        [ViewVariables(VVAccess.ReadWrite)]
        public RSI? BaseRSI
        {
            get => _baseRsi;
            set
            {
                _baseRsi = value;
                if (Layers == null || value == null)
                {
                    return;
                }

                for (var i = 0; i < Layers.Count; i++)
                {
                    var layer = Layers[i];
                    if (!layer.State.IsValid || layer.RSI != null)
                    {
                        continue;
                    }

                    if (value.TryGetState(layer.State, out var state))
                    {
                        layer.AnimationTimeLeft = state.GetDelay(0);
                    }
                    else
                    {
                        Logger.ErrorS(LogCategory,
                            "Layer '{0}'no longer has state '{1}' due to base RSI change. Trace:\n{2}",
                            i, layer.State, Environment.StackTrace);
                        layer.Texture = null;
                    }
                }
            }
        }

        [ViewVariables(VVAccess.ReadWrite)]
        public bool ContainerOccluded { get; set; }

        [ViewVariables(VVAccess.ReadWrite)]
        public bool TreeUpdateQueued { get; set; }

        [ViewVariables(VVAccess.ReadWrite)]
        public ShaderInstance? PostShader { get; set; }

        [ViewVariables] private Dictionary<object, int> LayerMap = new();
        [ViewVariables] private bool _layerMapShared;
        [ViewVariables] private List<Layer> Layers = default!;

        [Dependency] private readonly IResourceCache resourceCache = default!;
        [Dependency] private readonly IPrototypeManager prototypes = default!;
        [Dependency] private readonly IReflectionManager reflectionManager = default!;

        [ViewVariables(VVAccess.ReadWrite)] public uint RenderOrder { get; set; }

        // TODO: this should absolutely not be static.
        private static ShaderInstance? _defaultShader;

        [ViewVariables]
        private ShaderInstance? DefaultShader => _defaultShader ??
                                                 (_defaultShader = prototypes
                                                     .Index<ShaderPrototype>("shaded")
                                                     .Instance());

        public const string LogCategory = "go.comp.sprite";
        const string LayerSerializationCache = "spritelayer";
        const string LayerMapSerializationCache = "spritelayermap";

        [ViewVariables(VVAccess.ReadWrite)] public bool IsInert { get; private set; }

        /// <summary>
        /// Update this sprite component to visibly match the current state of other at the time
        /// this is called. Does not keep them perpetually in sync.
        /// This does some deep copying thus exerts some gc pressure, so avoid this for hot code paths.
        /// </summary>
        public void CopyFrom(SpriteComponent other)
        {
            //deep copying things to avoid entanglement
            _baseRsi = other._baseRsi;
            _directional = other._directional;
            _visible = other._visible;
            _layerMapShared = other._layerMapShared;
            color = other.color;
            offset = other.offset;
            rotation = other.rotation;
            scale = other.scale;
            drawDepth = other.drawDepth;
            Layers = new List<Layer>(other.Layers.Count);
            foreach (var otherLayer in other.Layers)
            {
                Layers.Add(new Layer(otherLayer, this));
            }
            IsInert = other.IsInert;
            LayerMap = other.LayerMap.ToDictionary(entry => entry.Key,
                entry => entry.Value);
            if (other.PostShader != null)
            {
                // only need to copy the shader if it's mutable
                PostShader = other.PostShader.Mutable ? other.PostShader.Duplicate() : other.PostShader;
            }
            else
            {
                PostShader = null;
            }

            RenderOrder = other.RenderOrder;
        }

        /// <inheritdoc />
        public void LayerMapSet(object key, int layer)
        {
            if (layer < 0 || layer >= Layers.Count)
            {
                throw new ArgumentOutOfRangeException();
            }

            _layerMapEnsurePrivate();
            LayerMap.Add(key, layer);
        }

        /// <inheritdoc />
        public void LayerMapRemove(object key)
        {
            _layerMapEnsurePrivate();
            LayerMap.Remove(key);
        }

        /// <inheritdoc />
        public int LayerMapGet(object key)
        {
            return LayerMap[key];
        }

        /// <inheritdoc />
        public bool LayerMapTryGet(object key, out int layer)
        {
            return LayerMap.TryGetValue(key, out layer);
        }

        private void _layerMapEnsurePrivate()
        {
            if (!_layerMapShared)
            {
                return;
            }

            LayerMap = LayerMap.ShallowClone();
            _layerMapShared = false;
        }

        public void LayerMapReserveBlank(object key)
        {
            if (LayerMapTryGet(key, out var _))
            {
                return;
            }

            LayerMapSet(key, AddBlankLayer());
        }

        public int AddBlankLayer(int? newIndex = null)
        {
            var layer = new Layer(this) {Visible = false};
            return AddLayer(layer, newIndex);
        }

        public int AddLayer(string texturePath, int? newIndex = null)
        {
            return AddLayer(new ResourcePath(texturePath), newIndex);
        }

        public int AddLayer(ResourcePath texturePath, int? newIndex = null)
        {
            if (!resourceCache.TryGetResource<TextureResource>(TextureRoot / texturePath, out var texture))
            {
                if (texturePath.Extension == "rsi")
                {
                    Logger.ErrorS(LogCategory,
                        "Expected texture but got rsi '{0}', did you mean 'sprite:' instead of 'texture:'?",
                        texturePath);
                }

                Logger.ErrorS(LogCategory, "Unable to load texture '{0}'. Trace:\n{1}", texturePath,
                    Environment.StackTrace);
            }

            return AddLayer(texture?.Texture, newIndex);
        }

        public int AddLayer(Texture? texture, int? newIndex = null)
        {
            var layer = new Layer(this) {Texture = texture};
            return AddLayer(layer, newIndex);
        }

        public int AddLayer(RSI.StateId stateId, int? newIndex = null)
        {
            var layer = new Layer(this) {State = stateId};
            if (BaseRSI != null && BaseRSI.TryGetState(stateId, out var state))
            {
                layer.AnimationTimeLeft = state.GetDelay(0);
            }
            else
            {
                Logger.ErrorS(LogCategory, "State does not exist in RSI: '{0}'. Trace:\n{1}", stateId,
                    Environment.StackTrace);
            }

            return AddLayer(layer, newIndex);
        }

        public int AddLayerState(string stateId, int? newIndex = null)
        {
            return AddLayer(new RSI.StateId(stateId), newIndex);
        }

        public int AddLayer(RSI.StateId stateId, string rsiPath, int? newIndex = null)
        {
            return AddLayer(stateId, new ResourcePath(rsiPath), newIndex);
        }

        public int AddLayerState(string stateId, string rsiPath, int? newIndex = null)
        {
            return AddLayer(new RSI.StateId(stateId), rsiPath, newIndex);
        }

        public int AddLayer(RSI.StateId stateId, ResourcePath rsiPath, int? newIndex = null)
        {
            if (!resourceCache.TryGetResource<RSIResource>(TextureRoot / rsiPath, out var res))
            {
                Logger.ErrorS(LogCategory, "Unable to load RSI '{0}'. Trace:\n{1}", rsiPath, Environment.StackTrace);
            }

            return AddLayer(stateId, res?.RSI);
        }

        public int AddLayerState(string stateId, ResourcePath rsiPath, int? newIndex = null)
        {
            return AddLayer(new RSI.StateId(stateId), rsiPath, newIndex);
        }

        public int AddLayer(RSI.StateId stateId, RSI? rsi, int? newIndex = null)
        {
            var layer = new Layer(this) {State = stateId, RSI = rsi};
            if (rsi != null && rsi.TryGetState(stateId, out var state))
            {
                layer.AnimationTimeLeft = state.GetDelay(0);
            }
            else
            {
                Logger.ErrorS(LogCategory, "State does not exist in RSI: '{0}'. Trace:\n{1}", stateId,
                    Environment.StackTrace);
            }

            return AddLayer(layer, newIndex);
        }

        public int AddLayerState(string stateId, RSI rsi, int? newIndex = null)
        {
            return AddLayer(new RSI.StateId(stateId), rsi, newIndex);
        }

        public int AddLayer(SpriteSpecifier specifier, int? newIndex = null)
        {
            switch (specifier)
            {
                case SpriteSpecifier.Texture tex:
                    return AddLayer(tex.TexturePath, newIndex);

                case SpriteSpecifier.Rsi rsi:
                    return AddLayerState(rsi.RsiState, rsi.RsiPath, newIndex);

                default:
                    throw new NotImplementedException();
            }
        }

        private int AddLayer(Layer layer, int? newIndex)
        {
            int index;
            if (newIndex.HasValue)
            {
                Layers.Insert(newIndex.Value, layer);
                foreach (var kv in LayerMap)
                {
                    if (kv.Value >= newIndex.Value)
                    {
                        LayerMap[kv.Key] = kv.Value + 1;
                    }
                }

                index = newIndex.Value;
            }
            else
            {
                Layers.Add(layer);
                index = Layers.Count - 1;
            }

            UpdateIsInert();
            return index;
        }

        public void RemoveLayer(int layer)
        {
            if (Layers.Count <= layer)
            {
                Logger.ErrorS(LogCategory, "Layer with index '{0}' does not exist, cannot remove! Trace:\n{1}", layer,
                    Environment.StackTrace);
                return;
            }

            Layers.RemoveAt(layer);
            foreach (var kv in LayerMap)
            {
                if (kv.Value == layer)
                {
                    LayerMap.Remove(kv.Key);
                }

                else if (kv.Value > layer)
                {
                    LayerMap[kv.Key] = kv.Value - 1;
                }
            }

            UpdateIsInert();
        }

        public void RemoveLayer(object layerKey)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot remove! Trace:\n{1}", layerKey,
                    Environment.StackTrace);
                return;
            }

            RemoveLayer(layer);
        }

        public void LayerSetShader(int layer, ShaderInstance? shader)
        {
            if (Layers.Count <= layer)
            {
                Logger.ErrorS(LogCategory, "Layer with index '{0}' does not exist, cannot set shader! Trace:\n{1}",
                    layer, Environment.StackTrace);
                return;
            }

            var theLayer = Layers[layer];
            theLayer.Shader = shader;
        }

        public void LayerSetShader(object layerKey, ShaderInstance shader)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot set shader! Trace:\n{1}",
                    layerKey, Environment.StackTrace);
                return;
            }

            LayerSetShader(layer, shader);
        }

        public void LayerSetShader(int layer, string shaderName)
        {
            if (!prototypes.TryIndex<ShaderPrototype>(shaderName, out var prototype))
            {
                Logger.ErrorS(LogCategory, "Shader prototype '{0}' does not exist. Trace:\n{1}", shaderName,
                    Environment.StackTrace);
            }

            // This will set the shader to null if it does not exist.
            LayerSetShader(layer, prototype?.Instance());
        }

        public void LayerSetShader(object layerKey, string shaderName)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot set shader! Trace:\n{1}",
                    layerKey, Environment.StackTrace);
                return;
            }

            LayerSetShader(layer, shaderName);
        }

        public void LayerSetSprite(int layer, SpriteSpecifier specifier)
        {
            if (Layers.Count <= layer)
            {
                Logger.ErrorS(LogCategory, "Layer with index '{0}' does not exist, cannot set sprite! Trace:\n{1}",
                    layer, Environment.StackTrace);
                return;
            }

            switch (specifier)
            {
                case SpriteSpecifier.Texture tex:
                    LayerSetTexture(layer, tex.TexturePath);
                    break;
                case SpriteSpecifier.Rsi rsi:
                    LayerSetState(layer, rsi.RsiState, rsi.RsiPath);
                    break;
                default:
                    throw new NotImplementedException();
            }
        }

        public void LayerSetSprite(object layerKey, SpriteSpecifier specifier)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot set sprite! Trace:\n{1}",
                    layerKey, Environment.StackTrace);
                return;
            }

            LayerSetSprite(layer, specifier);
        }

        public void LayerSetTexture(int layer, Texture? texture)
        {
            if (Layers.Count <= layer)
            {
                Logger.ErrorS(LogCategory, "Layer with index '{0}' does not exist, cannot set texture! Trace:\n{1}",
                    layer, Environment.StackTrace);
                return;
            }

            var theLayer = Layers[layer];
            theLayer.SetTexture(texture);
        }

        public void LayerSetTexture(object layerKey, Texture texture)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot set texture! Trace:\n{1}",
                    layerKey, Environment.StackTrace);
                return;
            }

            LayerSetTexture(layer, texture);
        }

        public void LayerSetTexture(int layer, string texturePath)
        {
            LayerSetTexture(layer, new ResourcePath(texturePath));
        }

        public void LayerSetTexture(object layerKey, string texturePath)
        {
            LayerSetTexture(layerKey, new ResourcePath(texturePath));
        }

        public void LayerSetTexture(int layer, ResourcePath texturePath)
        {
            if (!resourceCache.TryGetResource<TextureResource>(TextureRoot / texturePath, out var texture))
            {
                if (texturePath.Extension == "rsi")
                {
                    Logger.ErrorS(LogCategory,
                        "Expected texture but got rsi '{0}', did you mean 'sprite:' instead of 'texture:'?",
                        texturePath);
                }

                Logger.ErrorS(LogCategory, "Unable to load texture '{0}'. Trace:\n{1}", texturePath,
                    Environment.StackTrace);
            }

            LayerSetTexture(layer, texture?.Texture);
        }

        public void LayerSetTexture(object layerKey, ResourcePath texturePath)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot set texture! Trace:\n{1}",
                    layerKey, Environment.StackTrace);
                return;
            }

            LayerSetTexture(layer, texturePath);
        }

        public void LayerSetState(int layer, RSI.StateId stateId)
        {
            if (Layers.Count <= layer)
            {
                Logger.ErrorS(LogCategory, "Layer with index '{0}' does not exist, cannot set state! Trace:\n{1}",
                    layer, Environment.StackTrace);
                return;
            }

            var theLayer = Layers[layer];
            theLayer.SetState(stateId);
        }

        public void LayerSetState(object layerKey, RSI.StateId stateId)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot set state! Trace:\n{1}",
                    layerKey, Environment.StackTrace);
                return;
            }

            LayerSetState(layer, stateId);
        }

        public void LayerSetState(int layer, RSI.StateId stateId, RSI? rsi)
        {
            if (Layers.Count <= layer)
            {
                Logger.ErrorS(LogCategory, "Layer with index '{0}' does not exist, cannot set state! Trace:\n{1}",
                    layer, Environment.StackTrace);
                return;
            }

            var theLayer = Layers[layer];
            theLayer.State = stateId;
            theLayer.RSI = rsi;
            var actualRsi = theLayer.RSI ?? BaseRSI;
            if (actualRsi == null)
            {
                Logger.ErrorS(LogCategory, "No RSI to pull new state from! Trace:\n{0}", Environment.StackTrace);
                theLayer.Texture = null;
            }
            else
            {
                if (actualRsi.TryGetState(stateId, out var state))
                {
                    theLayer.AnimationFrame = 0;
                    theLayer.AnimationTime = 0;
                    theLayer.AnimationTimeLeft = state.GetDelay(0);
                }
                else
                {
                    Logger.ErrorS(LogCategory, "State '{0}' does not exist in RSI. Trace:\n{1}", stateId,
                        Environment.StackTrace);
                    theLayer.Texture = null;
                }
            }
        }

        public void LayerSetState(object layerKey, RSI.StateId stateId, RSI rsi)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot set state! Trace:\n{1}",
                    layerKey, Environment.StackTrace);
                return;
            }

            LayerSetState(layer, stateId, rsi);
        }

        public void LayerSetState(int layer, RSI.StateId stateId, string rsiPath)
        {
            LayerSetState(layer, stateId, new ResourcePath(rsiPath));
        }

        public void LayerSetState(object layerKey, RSI.StateId stateId, string rsiPath)
        {
            LayerSetState(layerKey, stateId, new ResourcePath(rsiPath));
        }

        public void LayerSetState(int layer, RSI.StateId stateId, ResourcePath rsiPath)
        {
            if (!resourceCache.TryGetResource<RSIResource>(TextureRoot / rsiPath, out var res))
            {
                Logger.ErrorS(LogCategory, "Unable to load RSI '{0}'. Trace:\n{1}", rsiPath, Environment.StackTrace);
            }

            LayerSetState(layer, stateId, res?.RSI);
        }

        public void LayerSetState(object layerKey, RSI.StateId stateId, ResourcePath rsiPath)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot set state! Trace:\n{1}",
                    layerKey, Environment.StackTrace);
                return;
            }

            LayerSetState(layer, stateId, rsiPath);
        }

        public void LayerSetRSI(int layer, RSI? rsi)
        {
            if (Layers.Count <= layer)
            {
                Logger.ErrorS(LogCategory, "Layer with index '{0}' does not exist, cannot set RSI! Trace:\n{1}", layer,
                    Environment.StackTrace);
                return;
            }

            var theLayer = Layers[layer];
            theLayer.SetRsi(rsi);
        }

        public void LayerSetRSI(object layerKey, RSI rsi)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot set RSI! Trace:\n{1}", layerKey,
                    Environment.StackTrace);
                return;
            }

            LayerSetRSI(layer, rsi);
        }

        public void LayerSetRSI(int layer, string rsiPath)
        {
            LayerSetRSI(layer, new ResourcePath(rsiPath));
        }

        public void LayerSetRSI(object layerKey, string rsiPath)
        {
            LayerSetRSI(layerKey, new ResourcePath(rsiPath));
        }

        public void LayerSetRSI(int layer, ResourcePath rsiPath)
        {
            if (!resourceCache.TryGetResource<RSIResource>(TextureRoot / rsiPath, out var res))
            {
                Logger.ErrorS(LogCategory, "Unable to load RSI '{0}'. Trace:\n{1}", rsiPath, Environment.StackTrace);
            }

            LayerSetRSI(layer, res?.RSI);
        }

        public void LayerSetRSI(object layerKey, ResourcePath rsiPath)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot set RSI! Trace:\n{1}", layerKey,
                    Environment.StackTrace);
                return;
            }

            LayerSetRSI(layer, rsiPath);
        }

        public void LayerSetScale(int layer, Vector2 scale)
        {
            if (Layers.Count <= layer)
            {
                Logger.ErrorS(LogCategory, "Layer with index '{0}' does not exist, cannot set scale! Trace:\n{1}",
                    layer, Environment.StackTrace);
                return;
            }

            var theLayer = Layers[layer];
            theLayer.Scale = scale;
        }

        public void LayerSetScale(object layerKey, Vector2 scale)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot set scale! Trace:\n{1}",
                    layerKey, Environment.StackTrace);
                return;
            }

            LayerSetScale(layer, scale);
        }


        public void LayerSetRotation(int layer, Angle rotation)
        {
            if (Layers.Count <= layer)
            {
                Logger.ErrorS(LogCategory, "Layer with index '{0}' does not exist, cannot set rotation! Trace:\n{1}",
                    layer, Environment.StackTrace);
                return;
            }

            var theLayer = Layers[layer];
            theLayer.Rotation = rotation;
        }

        public void LayerSetRotation(object layerKey, Angle rotation)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot set rotation! Trace:\n{1}",
                    layerKey, Environment.StackTrace);
                return;
            }

            LayerSetRotation(layer, rotation);
        }

        public void LayerSetVisible(int layer, bool visible)
        {
            if (Layers.Count <= layer)
            {
                Logger.ErrorS(LogCategory, "Layer with index '{0}' does not exist, cannot set visibility! Trace:\n{1}",
                    layer, Environment.StackTrace);
                return;
            }

            Layers[layer].SetVisible(visible);
        }

        public void LayerSetVisible(object layerKey, bool visible)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot set visibility! Trace:\n{1}",
                    layerKey, Environment.StackTrace);
                return;
            }

            LayerSetVisible(layer, visible);
        }

        public void LayerSetColor(int layer, Color color)
        {
            if (Layers.Count <= layer)
            {
                Logger.ErrorS(LogCategory, "Layer with index '{0}' does not exist, cannot set color! Trace:\n{1}",
                    layer, Environment.StackTrace);
                return;
            }

            var theLayer = Layers[layer];
            theLayer.Color = color;
        }

        public void LayerSetColor(object layerKey, Color color)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot set color! Trace:\n{1}",
                    layerKey, Environment.StackTrace);
                return;
            }

            LayerSetColor(layer, color);
        }

        public void LayerSetDirOffset(int layer, DirectionOffset offset)
        {
            if (Layers.Count <= layer)
            {
                Logger.ErrorS(LogCategory, "Layer with index '{0}' does not exist, cannot set dir offset! Trace:\n{1}",
                    layer, Environment.StackTrace);
                return;
            }

            var theLayer = Layers[layer];
            theLayer.DirOffset = offset;
        }

        public void LayerSetDirOffset(object layerKey, DirectionOffset offset)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot set dir offset! Trace:\n{1}",
                    layerKey, Environment.StackTrace);
                return;
            }

            LayerSetDirOffset(layer, offset);
        }

        public void LayerSetAnimationTime(int layer, float animationTime)
        {
            if (Layers.Count <= layer)
            {
                Logger.ErrorS(LogCategory,
                    "Layer with index '{0}' does not exist, cannot set animation time! Trace:\n{1}",
                    layer, Environment.StackTrace);
                return;
            }

            Layers[layer].SetAnimationTime(animationTime);
        }

        public void LayerSetAnimationTime(object layerKey, float animationTime)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory,
                    "Layer with key '{0}' does not exist, cannot set animation time! Trace:\n{1}",
                    layerKey, Environment.StackTrace);
                return;
            }

            LayerSetAnimationTime(layer, animationTime);
        }

        public void LayerSetAutoAnimated(int layer, bool autoAnimated)
        {
            if (Layers.Count <= layer)
            {
                Logger.ErrorS(LogCategory,
                    "Layer with index '{0}' does not exist, cannot set auto animated! Trace:\n{1}",
                    layer, Environment.StackTrace);
                return;
            }

            Layers[layer].SetAutoAnimated(autoAnimated);
        }

        public void LayerSetAutoAnimated(object layerKey, bool autoAnimated)
        {
            if (!LayerMapTryGet(layerKey, out var layer))
            {
                Logger.ErrorS(LogCategory, "Layer with key '{0}' does not exist, cannot set auto animated! Trace:\n{1}",
                    layerKey, Environment.StackTrace);
                return;
            }

            LayerSetAutoAnimated(layer, autoAnimated);
        }

        /// <inheritdoc />
        public RSI.StateId LayerGetState(int layer)
        {
            if (Layers.Count <= layer)
            {
                Logger.ErrorS(LogCategory, "Layer with index '{0}' does not exist, cannot get state! Trace:\n{1}",
                    layer, Environment.StackTrace);
                return null;
            }

            var thelayer = Layers[layer];
            return thelayer.State;
        }

        public RSI? LayerGetActualRSI(int layer)
        {
            return this[layer].ActualRsi;
        }

        public RSI? LayerGetActualRSI(object layerKey)
        {
            return this[layerKey].ActualRsi;
        }

        public ISpriteLayer this[int layer] => Layers[layer];
        public ISpriteLayer this[Index layer] => Layers[layer];
        public ISpriteLayer this[object layerKey] => this[LayerMap[layerKey]];
        public IEnumerable<ISpriteLayer> AllLayers => Layers;

        internal void Render(DrawingHandleWorld drawingHandle, in Matrix3 worldTransform, Angle worldRotation,
            Direction? overrideDirection = null)
        {
            var angle = Rotation;
            if (Directional)
            {
                angle -= worldRotation;
            }
            else
            {
                angle -= new Angle(MathHelper.PiOver2);
            }

            var mOffset = Matrix3.CreateTranslation(Offset);
            var mRotation = Matrix3.CreateRotation(angle);
            Matrix3.Multiply(ref mRotation, ref mOffset, out var transform);

            // Only apply scale if needed.
            if(!Scale.EqualsApprox(Vector2.One)) transform.Multiply(Matrix3.CreateScale(Scale));

            transform.Multiply(worldTransform);

            RenderInternal(drawingHandle, worldRotation, overrideDirection, transform);
        }

        internal void Render(DrawingHandleWorld drawingHandle, Angle worldRotation, Direction? overrideDirection = null)
        {
            RenderInternal(drawingHandle, worldRotation, overrideDirection, Matrix3.Identity);
        }

        private void RenderInternal(DrawingHandleWorld drawingHandle, Angle worldRotation, Direction? overrideDirection,
            in Matrix3 transform)
        {
            drawingHandle.SetTransform(transform);

            foreach (var layer in Layers)
            {
                if (!layer.Visible)
                {
                    continue;
                }

                // TODO: Implement layer-specific rotation and scale.
                // Oh and when you do update Layer.LocalToLayer so content doesn't break.

                var texture = GetRenderTexture(layer, worldRotation, overrideDirection);

                if (layer.Shader != null)
                {
                    drawingHandle.UseShader(layer.Shader);
                }

                drawingHandle.DrawTexture(texture, -(Vector2) texture.Size / (2f * EyeManager.PixelsPerMeter),
                    color * layer.Color);

                if (layer.Shader != null)
                {
                    drawingHandle.UseShader(null);
                }
            }
        }

        private Texture GetRenderTexture(Layer layer, Angle worldRotation, Direction? overrideDirection)
        {
            var texture = layer.Texture;

            if (layer.State.IsValid)
            {
                // Pull texture from RSI state instead.
                var rsi = layer.RSI ?? BaseRSI;
                if (rsi == null || !rsi.TryGetState(layer.State, out var state))
                {
                    state = GetFallbackState(resourceCache);
                }

                var layerSpecificDir = layer.EffectiveDirection(state, worldRotation, overrideDirection);
                texture = state.GetFrame(layerSpecificDir, layer.AnimationFrame);
            }

            texture ??= resourceCache.GetFallback<TextureResource>().Texture;
            return texture;
        }

        public override void ExposeData(ObjectSerializer serializer)
        {
            base.ExposeData(serializer);

            serializer.DataFieldCached(ref scale, "scale", Vector2.One);
            serializer.DataFieldCached(ref rotation, "rotation", Angle.Zero);
            serializer.DataFieldCached(ref offset, "offset", Vector2.Zero);
            serializer.DataFieldCached(ref drawDepth, "drawdepth", DrawDepthTag.Default,
                WithFormat.Constants<DrawDepthTag>());
            serializer.DataFieldCached(ref color, "color", Color.White);
            serializer.DataFieldCached(ref _directional, "directional", true);
            serializer.DataFieldCached(ref _visible, "visible", true);

            // TODO: Writing?
            if (!serializer.Reading)
            {
                return;
            }

            {
                var rsi = serializer.ReadDataField<string?>("sprite", null);
                if (!string.IsNullOrWhiteSpace(rsi))
                {
                    var rsiPath = TextureRoot / rsi;
                    try
                    {
                        BaseRSI = resourceCache.GetResource<RSIResource>(rsiPath).RSI;
                    }
                    catch (Exception e)
                    {
                        Logger.ErrorS(LogCategory, "Unable to load RSI '{0}'. Trace:\n{1}", rsiPath, e);
                    }
                }
            }

            List<Layer> CloneLayers(List<Layer> source)
            {
                var clone = new List<Layer>(source.Count);
                foreach (var layer in source)
                {
                    clone.Add(new Layer(layer, this));
                }

                return clone;
            }

            if (serializer.TryGetCacheData<List<Layer>>(LayerSerializationCache, out var layers))
            {
                LayerMap = serializer.GetCacheData<Dictionary<object, int>>(LayerMapSerializationCache);
                _layerMapShared = true;
                Layers = CloneLayers(layers);
                UpdateIsInert();
                return;
            }

            layers = new List<Layer>();

            var layerMap = new Dictionary<object, int>();

            var layerData =
                serializer.ReadDataField("layers", new List<PrototypeLayerData>());

            if(layerData.Count == 0){
                var baseState = serializer.ReadDataField<string?>("state", null);
                var texturePath = serializer.ReadDataField<string?>("texture", null);

                if (baseState != null || texturePath != null)
                {
                    layerData.Insert(0, new PrototypeLayerData
                    {
                        TexturePath = string.IsNullOrWhiteSpace(texturePath) ? null : texturePath,
                        State = string.IsNullOrWhiteSpace(baseState) ? null : baseState,
                        Color = Color.White,
                        Scale = Vector2.One,
                        Visible = true,
                    });
                }
            }

            foreach (var layerDatum in layerData)
            {
                var anyTextureAttempted = false;
                var layer = new Layer(this);
                if (!string.IsNullOrWhiteSpace(layerDatum.RsiPath))
                {
                    var path = TextureRoot / layerDatum.RsiPath;
                    try
                    {
                        layer.RSI = resourceCache.GetResource<RSIResource>(path).RSI;
                    }
                    catch
                    {
                        Logger.ErrorS(LogCategory, "Unable to load layer RSI '{0}'.", path);
                    }
                }

                if (!string.IsNullOrWhiteSpace(layerDatum.State))
                {
                    anyTextureAttempted = true;
                    var theRsi = layer.RSI ?? BaseRSI;
                    if (theRsi == null)
                    {
                        Logger.ErrorS(LogCategory,
                            "Layer has no RSI to load states from."
                            + "cannot use 'state' property. Prototype: '{0}'", Owner.Prototype?.ID);
                    }
                    else
                    {
                        var stateid = new RSI.StateId(layerDatum.State);
                        layer.State = stateid;
                        if (theRsi.TryGetState(stateid, out var state))
                        {
                            // Always use south because this layer will be cached in the serializer.
                            layer.AnimationTimeLeft = state.GetDelay(0);
                        }
                        else
                        {
                            Logger.ErrorS(LogCategory,
                                $"State '{stateid}' not found in RSI: '{theRsi.Path}'.",
                                stateid);
                        }
                    }
                }

                if (!string.IsNullOrWhiteSpace(layerDatum.TexturePath))
                {
                    anyTextureAttempted = true;
                    if (layer.State.IsValid)
                    {
                        Logger.ErrorS(LogCategory,
                            "Cannot specify 'texture' on a layer if it has an RSI state specified."
                        );
                    }
                    else
                    {
                        layer.Texture =
                            resourceCache.GetResource<TextureResource>(TextureRoot / layerDatum.TexturePath);
                    }
                }

                if (!string.IsNullOrWhiteSpace(layerDatum.Shader))
                {
                    if (prototypes.TryIndex<ShaderPrototype>(layerDatum.Shader, out var prototype))
                    {
                        layer.Shader = prototype.Instance();
                    }
                    else
                    {
                        Logger.ErrorS(LogCategory,
                            "Shader prototype '{0}' does not exist. Prototype: '{1}'",
                            layerDatum.Shader, Owner.Prototype?.ID);
                    }
                }

                layer.Color = layerDatum.Color;
                layer.Rotation = layerDatum.Rotation;
                // If neither state: nor texture: were provided we assume that they want a blank invisible layer.
                layer.Visible = anyTextureAttempted && layerDatum.Visible;
                layer.Scale = layerDatum.Scale;

                layers.Add(layer);

                if (layerDatum.MapKeys != null)
                {
                    var index = layers.Count - 1;
                    foreach (var keyString in layerDatum.MapKeys)
                    {
                        object key;
                        if (reflectionManager.TryParseEnumReference(keyString, out var @enum))
                        {
                            key = @enum;
                        }
                        else
                        {
                            key = keyString;
                        }

                        if (layerMap.ContainsKey(key))
                        {
                            Logger.ErrorS(LogCategory, "Duplicate layer map key definition: {0}", key);
                            continue;
                        }

                        layerMap.Add(key, index);
                    }
                }
            }

            Layers = layers;
            LayerMap = layerMap;
            _layerMapShared = true;
            serializer.SetCacheData(LayerSerializationCache, CloneLayers(Layers));
            serializer.SetCacheData(LayerMapSerializationCache, layerMap);
            UpdateIsInert();
        }

        public override void OnRemove()
        {
            base.OnRemove();

            var map = Owner.Transform.MapID;
            if (map != MapId.Nullspace)
            {
                Owner.EntityManager.EventBus.RaiseEvent(EventSource.Local,
                    new RenderTreeRemoveSpriteMessage(this, map));
            }
        }

        public void FrameUpdate(float delta)
        {
            foreach (var t in Layers)
            {
                var layer = t;
                // Since StateId is a struct, we can't null-check it directly.
                if (!layer.State.IsValid || !layer.Visible || !layer.AutoAnimated)
                {
                    continue;
                }

                var rsi = layer.RSI ?? BaseRSI;
                if (rsi == null || !rsi.TryGetState(layer.State, out var state))
                {
                    state = GetFallbackState(resourceCache);
                }

                if (!state.IsAnimated)
                {
                    continue;
                }

                layer.AnimationTime += delta;
                layer.AnimationTimeLeft -= delta;
                _advanceFrameAnimation(layer, state);
            }
        }

        private static void _advanceFrameAnimation(Layer layer, RSI.State state)
        {
            var delayCount = state.DelayCount;
            while (layer.AnimationTimeLeft < 0)
            {
                layer.AnimationFrame += 1;

                if (layer.AnimationFrame >= delayCount)
                {
                    layer.AnimationFrame = 0;
                    layer.AnimationTime = -layer.AnimationTimeLeft;
                }

                layer.AnimationTimeLeft += state.GetDelay(layer.AnimationFrame);
            }
        }

        public override void HandleComponentState(ComponentState? curState, ComponentState? nextState)
        {
            if (curState == null)
                return;

            var thestate = (SpriteComponentState) curState;

            Visible = thestate.Visible;
            DrawDepth = thestate.DrawDepth;
            Scale = thestate.Scale;
            Rotation = thestate.Rotation;
            Offset = thestate.Offset;
            Color = thestate.Color;
            Directional = thestate.Directional;
            RenderOrder = thestate.RenderOrder;

            if (thestate.BaseRsiPath != null && BaseRSI != null)
            {
                if (resourceCache.TryGetResource<RSIResource>(TextureRoot / thestate.BaseRsiPath, out var res))
                {
                    if (BaseRSI != res.RSI)
                    {
                        BaseRSI = res.RSI;
                    }
                }
                else
                {
                    Logger.ErrorS(LogCategory, "Hey server, RSI '{0}' doesn't exist.", thestate.BaseRsiPath);
                }
            }

            // Maybe optimize this to NOT full clear.
            Layers.Clear();
            for (var i = 0; i < thestate.Layers.Count; i++)
            {
                var netlayer = thestate.Layers[i];
                var layer = new Layer(this)
                {
                    // These are easy so do them here.
                    Scale = netlayer.Scale,
                    Rotation = netlayer.Rotation,
                    Visible = netlayer.Visible,
                    Color = netlayer.Color
                };
                Layers.Add(layer);

                // Using the public API to handle errors.
                // Probably slow as crap.
                // Who am I kidding, DEFINITELY.
                if (netlayer.Shader != null)
                {
                    LayerSetShader(i, netlayer.Shader);
                }

                if (netlayer.RsiPath != null)
                {
                    LayerSetRSI(i, netlayer.RsiPath);
                }

                if (netlayer.TexturePath != null)
                {
                    LayerSetTexture(i, netlayer.TexturePath);
                }
                else if (netlayer.State != null)
                {
                    LayerSetState(i, netlayer.State);
                }
            }
        }

        private RSI.State.Direction GetDir(RSI.State.DirectionType type, Angle worldRotation)
        {
            if (!Directional)

            {
                return RSI.State.Direction.South;
            }

            var angle = new Angle(worldRotation);
            return angle.GetDir().Convert(type);
        }

        private void UpdateIsInert()
        {
            IsInert = true;

            foreach (var layer in Layers)
            {
                // Since StateId is a struct, we can't null-check it directly.
                if (!layer.State.IsValid || !layer.Visible || !layer.AutoAnimated)
                {
                    continue;
                }

                var rsi = layer.RSI ?? BaseRSI;
                if (rsi == null || !rsi.TryGetState(layer.State, out var state))
                {
                    state = GetFallbackState(resourceCache);
                }

                if (state.IsAnimated)
                {
                    IsInert = false;
                    break;
                }
            }
        }

        internal static RSI.State GetFallbackState(IResourceCache cache)
        {
            var rsi = cache.GetResource<RSIResource>("/Textures/error.rsi").RSI;
            return rsi["error"];
        }

        private static RSI.State.Direction OffsetRsiDir(RSI.State.Direction dir, DirectionOffset offset)
        {
            // There is probably a better way to do this.
            // Eh.
            switch (offset)
            {
                case DirectionOffset.None:
                    return dir;
                case DirectionOffset.Clockwise:
                    return dir switch
                    {
                        RSI.State.Direction.North => RSI.State.Direction.East,
                        RSI.State.Direction.East => RSI.State.Direction.South,
                        RSI.State.Direction.South => RSI.State.Direction.West,
                        RSI.State.Direction.West => RSI.State.Direction.North,
                        _ => throw new NotImplementedException()
                    };
                case DirectionOffset.CounterClockwise:
                    return dir switch
                    {
                        RSI.State.Direction.North => RSI.State.Direction.West,
                        RSI.State.Direction.East => RSI.State.Direction.North,
                        RSI.State.Direction.South => RSI.State.Direction.East,
                        RSI.State.Direction.West => RSI.State.Direction.South,
                        _ => throw new NotImplementedException()
                    };
                case DirectionOffset.Flip:
                    switch (dir)
                    {
                        case RSI.State.Direction.North:
                            return RSI.State.Direction.South;
                        case RSI.State.Direction.East:
                            return RSI.State.Direction.West;
                        case RSI.State.Direction.South:
                            return RSI.State.Direction.North;
                        case RSI.State.Direction.West:
                            return RSI.State.Direction.East;
                        default:
                            throw new NotImplementedException();
                    }
                default:
                    throw new NotImplementedException();
            }
        }

        public string GetDebugString()
        {
            var builder = new StringBuilder();
            builder.AppendFormat(
                "vis/depth/scl/rot/ofs/col/diral/dir: {0}/{1}/{2}/{3}/{4}/{5}/{6}/{7}\n",
                Visible, DrawDepth, Scale, Rotation, Offset,
                Color, Directional, GetDir(RSI.State.DirectionType.Dir8, Owner.Transform.WorldRotation)
            );

            foreach (var layer in Layers)
            {
                builder.AppendFormat(
                    "shad/tex/rsi/state/ant/anf/scl/rot/vis/col/dofs: {0}/{1}/{2}/{3}/{4}/{5}/{6}/{7}/{8}/{9}/{10}\n",
                    // These are references and don't include useful data for knowing where they came from, sadly.
                    // "is one set" is better than nothing at least.
                    layer.Shader != null, layer.Texture != null, layer.RSI != null,
                    layer.State,
                    layer.AnimationTimeLeft, layer.AnimationFrame, layer.Scale, layer.Rotation, layer.Visible,
                    layer.Color, layer.DirOffset
                );
            }

            return builder.ToString();
        }

        /// <summary>
        ///     Enum to "offset" a cardinal direction.
        /// </summary>
        public enum DirectionOffset : byte
        {
            /// <summary>
            ///     No offset.
            /// </summary>
            None = 0,

            /// <summary>
            ///     Rotate direction clockwise. (North -> East, etc...)
            /// </summary>
            Clockwise = 1,

            /// <summary>
            ///     Rotate direction counter-clockwise. (North -> West, etc...)
            /// </summary>
            CounterClockwise = 2,

            /// <summary>
            ///     Rotate direction 180 degrees, so flip. (North -> South, etc...)
            /// </summary>
            Flip = 3,
        }

        private class Layer : ISpriteLayer
        {
            [ViewVariables] private readonly SpriteComponent _parent;

            [ViewVariables] public ShaderInstance? Shader;
            [ViewVariables] public Texture? Texture;

            [ViewVariables] public RSI? RSI;
            [ViewVariables] public RSI.StateId State;
            [ViewVariables] public float AnimationTimeLeft;
            [ViewVariables] public float AnimationTime;
            [ViewVariables] public int AnimationFrame;

            [ViewVariables(VVAccess.ReadWrite)]
            public Vector2 Scale { get; set; } = Vector2.One;
            [ViewVariables(VVAccess.ReadWrite)]
            public Angle Rotation { get; set; }
            [ViewVariables(VVAccess.ReadWrite)]
            public bool Visible = true;
            [ViewVariables(VVAccess.ReadWrite)]
            public Color Color { get; set; } = Color.White;
            [ViewVariables(VVAccess.ReadWrite)]
            public bool AutoAnimated = true;
            [ViewVariables]
            public DirectionOffset DirOffset { get; set; }
            [ViewVariables]
            public RSI? ActualRsi => RSI ?? _parent.BaseRSI;

            public Layer(SpriteComponent parent)
            {
                _parent = parent;
            }

            public Layer(Layer toClone, SpriteComponent parentSprite)
            {
                _parent = parentSprite;
                if (toClone.Shader != null)
                {
                    Shader = toClone.Shader.Mutable ? toClone.Shader.Duplicate() : toClone.Shader;
                }
                Texture = toClone.Texture;
                RSI = toClone.RSI;
                State = toClone.State;
                AnimationTimeLeft = toClone.AnimationTimeLeft;
                AnimationTime = toClone.AnimationTime;
                AnimationFrame = toClone.AnimationFrame;
                Scale = toClone.Scale;
                Rotation = toClone.Rotation;
                Visible = toClone.Visible;
                Color = toClone.Color;
                DirOffset = toClone.DirOffset;
                AutoAnimated = toClone.AutoAnimated;
            }

            RSI? ISpriteLayer.Rsi { get => RSI; set => SetRsi(value); }
            RSI.StateId ISpriteLayer.RsiState { get => State; set => SetState(value); }
            Texture? ISpriteLayer.Texture { get => Texture; set => SetTexture(value); }

            bool ISpriteLayer.Visible
            {
                get => Visible;
                set => SetVisible(value);
            }

            float ISpriteLayer.AnimationTime
            {
                get => AnimationTime;
                set => SetAnimationTime(value);
            }

            int ISpriteLayer.AnimationFrame => AnimationFrame;

            bool ISpriteLayer.AutoAnimated
            {
                get => AutoAnimated;
                set => SetAutoAnimated(value);
            }

            public RSI.State.Direction EffectiveDirection(Angle worldRotation)
            {
                if (State == default)
                {
                    return default;
                }

                var rsi = ActualRsi;
                if (rsi == null)
                {
                    return default;
                }

                if (rsi.TryGetState(State, out var state))
                {
                    return EffectiveDirection(state, worldRotation, null);
                }

                return default;
            }

            public Vector2 LocalToLayer(Vector2 localPos)
            {
                // TODO: scale & rotation for layers is currently unimplemented.
                return localPos;
            }

            public RSI.State.Direction EffectiveDirection(RSI.State state, Angle worldRotation,
                Direction? overrideDirection)
            {
                if (state.Directions == RSI.State.DirectionType.Dir1)
                {
                    return RSI.State.Direction.South;
                }
                else
                {
                    RSI.State.Direction dir;
                    if (overrideDirection != null)
                    {
                        dir = overrideDirection.Value.Convert(state.Directions);
                    }
                    else
                    {
                        dir = _parent.GetDir(state.Directions, worldRotation);
                    }

                    return OffsetRsiDir(dir, DirOffset);
                }
            }

            public void SetAnimationTime(float animationTime)
            {
                if (!State.IsValid)
                {
                    return;
                }

                var theLayerRSI = ActualRsi;
                if (theLayerRSI == null)
                {
                    return;
                }

                var state = theLayerRSI[State];
                if (animationTime > AnimationTime)
                {
                    // Handle advancing differently from going backwards.
                    AnimationTimeLeft -= (animationTime - AnimationTime);
                }
                else
                {
                    // Going backwards we re-calculate from zero.
                    // Definitely possible to optimize this for going backwards but I'm too lazy to figure that out.
                    AnimationTimeLeft = -animationTime + state.GetDelay(0);
                    AnimationFrame = 0;
                }

                AnimationTime = animationTime;
                // After setting timing data correctly, run advance to get to the correct frame.
                _advanceFrameAnimation(this, state);
            }

            public void SetAutoAnimated(bool value)
            {
                AutoAnimated = value;

                _parent.UpdateIsInert();
            }

            public void SetVisible(bool value)
            {
                Visible = value;

                _parent.UpdateIsInert();
            }

            public void SetRsi(RSI? rsi)
            {
                RSI = rsi;
                if (!State.IsValid)
                {
                    return;
                }

                // Gotta do this because somebody might use null as argument (totally valid).
                var actualRsi = ActualRsi;
                if (actualRsi == null)
                {
                    Logger.ErrorS(LogCategory, "No RSI to pull new state from! Trace:\n{0}", Environment.StackTrace);
                    Texture = null;
                }
                else
                {
                    if (actualRsi.TryGetState(State, out var state))
                    {
                        AnimationTimeLeft = state.GetDelay(0);
                    }
                    else
                    {
                        Logger.ErrorS(LogCategory, "State '{0}' does not exist in set RSI. Trace:\n{1}", State,
                            Environment.StackTrace);
                        Texture = null;
                    }
                }

                _parent.UpdateIsInert();
            }

            public void SetState(RSI.StateId stateId)
            {
                if (State == stateId)
                {
                    return;
                }

                State = stateId;
                RSI.State? state;
                var rsi = ActualRsi;
                if (rsi == null)
                {
                    state = GetFallbackState(_parent.resourceCache);
                    Logger.ErrorS(LogCategory, "No RSI to pull new state from! Trace:\n{0}", Environment.StackTrace);
                }
                else
                {
                    if (!rsi.TryGetState(stateId, out state))
                    {
                        state = GetFallbackState(_parent.resourceCache);
                        Logger.ErrorS(LogCategory, "State '{0}' does not exist in RSI. Trace:\n{1}", stateId,
                            Environment.StackTrace);
                    }
                }

                AnimationFrame = 0;
                AnimationTime = 0;
                AnimationTimeLeft = state.GetDelay(0);

                _parent.UpdateIsInert();
            }

            public void SetTexture(Texture? texture)
            {
                State = default;
                Texture = texture;

                _parent.UpdateIsInert();
            }
        }

        void IAnimationProperties.SetAnimatableProperty(string name, object value)
        {
            if (!name.StartsWith("layer/"))
            {
                AnimationHelper.SetAnimatableProperty(this, name, value);
                return;
            }

            var delimiter = name.IndexOf("/", 6, StringComparison.Ordinal);
            var indexString = name.Substring(6, delimiter - 6);
            var index = int.Parse(indexString, CultureInfo.InvariantCulture);
            var layerProp = name.Substring(delimiter + 1);

            switch (layerProp)
            {
                case "texture":
                    LayerSetTexture(index, (string) value);
                    return;
                case "state":
                    LayerSetState(index, (string) value);
                    return;
                case "color":
                    LayerSetColor(index, (Color) value);
                    return;
                default:
                    throw new ArgumentException($"Unknown layer property '{layerProp}'");
            }
        }

        public IRsiStateLike? Icon
        {
            get
            {
                if (Layers.Count == 0) return null;

                var layer = Layers[0];

                var texture = layer.Texture;

                if (!layer.State.IsValid) return texture;

                // Pull texture from RSI state instead.
                var rsi = layer.RSI ?? BaseRSI;
                if (rsi == null || !rsi.TryGetState(layer.State, out var state))
                {
                    state = GetFallbackState(resourceCache);
                }

                return state;

            }
        }

        public static IEnumerable<IDirectionalTextureProvider> GetPrototypeTextures(EntityPrototype prototype, IResourceCache resourceCache)
        {
            var icon = IconComponent.GetPrototypeIcon(prototype, resourceCache);
            if (icon != null)
            {
                yield return icon;
                yield break;
            }

            if (!prototype.Components.TryGetValue("Sprite", out _))
            {
                yield return resourceCache.GetFallback<TextureResource>().Texture;
                yield break;
            }

            var dummy = new DummyIconEntity {Prototype = prototype};
            var spriteComponent = dummy.AddComponent<SpriteComponent>();

            if (prototype.Components.TryGetValue("Appearance", out _))
            {
                var appearanceComponent = dummy.AddComponent<AppearanceComponent>();
                foreach (var layer in appearanceComponent.Visualizers)
                {
                    layer.InitializeEntity(dummy);
                    layer.OnChangeData(appearanceComponent);
                }
            }

            var anyTexture = false;
            foreach (var layer in spriteComponent.AllLayers)
            {
                if (layer.Texture != null) yield return layer.Texture;
                if (!layer.RsiState.IsValid || !layer.Visible) continue;

                var rsi = layer.Rsi ?? spriteComponent.BaseRSI;
                if (rsi == null ||
                    !rsi.TryGetState(layer.RsiState, out var state))
                    continue;

                yield return state;
                anyTexture = true;
            }

            dummy.Delete();

            if (!anyTexture)
                yield return resourceCache.GetFallback<TextureResource>().Texture;

        }

        public static IRsiStateLike GetPrototypeIcon(EntityPrototype prototype, IResourceCache resourceCache)
        {
            var icon = IconComponent.GetPrototypeIcon(prototype, resourceCache);
            if (icon != null) return icon;

            if (!prototype.Components.ContainsKey("Sprite"))
            {
                return GetFallbackState(resourceCache);
            }

            var dummy = new DummyIconEntity {Prototype = prototype};
            var spriteComponent = dummy.AddComponent<SpriteComponent>();
            dummy.Delete();

            return spriteComponent.Icon ?? GetFallbackState(resourceCache);
        }

        #region DummyIconEntity
        private class DummyIconEntity : IEntity
        {
            public GameTick LastModifiedTick { get; } = GameTick.Zero;
            public IEntityManager EntityManager { get; } = null!;
            public string Name { get; set; } = string.Empty;
            public EntityUid Uid { get; } = EntityUid.Invalid;
            public bool Initialized { get; } = false;
            public bool Initializing { get; } = false;
            public bool Deleted { get; } = true;
            public bool Paused { get; set; }
            public EntityPrototype? Prototype { get; set; }

            public string Description { get; set; } = string.Empty;
            public bool IsValid()
            {
                return false;
            }

            public ITransformComponent Transform { get; } = null!;
            public IMetaDataComponent MetaData { get; } = null!;

            private Dictionary<Type, IComponent> _components = new();

            public T AddComponent<T>() where T : Component, new()
            {
                var typeFactory = IoCManager.Resolve<IDynamicTypeFactoryInternal>();
                var comp = (T) typeFactory.CreateInstanceUnchecked(typeof(T));
                _components[typeof(T)] = comp;
                comp.Owner = this;

                if (typeof(ISpriteComponent).IsAssignableFrom(typeof(T)))
                {
                    _components[typeof(ISpriteComponent)] = comp;
                }

                if (Prototype != null && Prototype.Components.TryGetValue(comp.Name, out var node))
                {
                    comp.ExposeData(YamlObjectSerializer.NewReader(node));
                }

                return comp;
            }

            public void RemoveComponent<T>()
            {
                _components.Remove(typeof(T));
            }

            public bool HasComponent<T>()
            {
                return _components.ContainsKey(typeof(T));
            }

            public bool HasComponent(Type type)
            {
                return _components.ContainsKey(type);
            }

            public T GetComponent<T>()
            {
                return (T) _components[typeof(T)];
            }

            public IComponent GetComponent(Type type)
            {
                return null!;
            }

            public IComponent GetComponent(uint netID)
            {
                return null!;
            }

            public bool TryGetComponent<T>([NotNullWhen(true)] out T? component) where T : class
            {
                component = null;
                if (!_components.TryGetValue(typeof(T), out var value)) return false;
                component = (T) value;
                return true;
            }

            public T? GetComponentOrNull<T>() where T : class
            {
                return null;
            }

            public bool TryGetComponent(Type type, [NotNullWhen(true)] out IComponent? component)
            {
                component = null;
                if (!_components.TryGetValue(type, out var value)) return false;
                component = value;
                return true;
            }

            public IComponent? GetComponentOrNull(Type type)
            {
                return null;
            }

            public bool TryGetComponent(uint netId, [NotNullWhen(true)]  out IComponent? component)
            {
                component = null;
                return false;
            }

            public IComponent? GetComponentOrNull(uint netId)
            {
                return null;
            }

            public void Shutdown()
            {
            }

            public void Delete()
            {
            }

            public IEnumerable<IComponent> GetAllComponents()
            {
                return Enumerable.Empty<IComponent>();
            }

            public IEnumerable<T> GetAllComponents<T>()
            {
                return Enumerable.Empty<T>();
            }

            public void SendMessage(IComponent? owner, ComponentMessage message)
            {
            }

            public void SendNetworkMessage(IComponent owner, ComponentMessage message, INetChannel? channel = null)
            {
            }

            public void Dirty()
            {
            }
        }
        #endregion
    }
}
