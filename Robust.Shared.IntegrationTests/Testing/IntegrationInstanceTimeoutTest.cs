using NUnit.Framework;

namespace Robust.UnitTesting.Shared.Testing;

[TestFixture]
internal sealed class IntegrationInstanceTimeoutTest : RobustIntegrationTest
{
    [Test]
    public async Task StalledCommandTimesOutWithDiagnostics()
    {
        var options = new ServerIntegrationOptions
        {
            Pool = false,
            WaitTimeout = TimeSpan.FromSeconds(10)
        };
        var server = StartServer(options);
        await server.WaitIdleAsync();
        options.WaitTimeout = TimeSpan.FromMilliseconds(100);

        var started = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var completed = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        using var release = new ManualResetEventSlim();

        server.Post(() =>
        {
            started.SetResult();
            release.Wait();
            completed.SetResult();
        });

        await started.Task.WaitAsync(TimeSpan.FromSeconds(10));

        try
        {
            var exception = Assert.ThrowsAsync<TimeoutException>(async () => await server.WaitIdleAsync());
            Assert.Multiple(() =>
            {
                Assert.That(exception!.Message, Does.Contain("ServerIntegrationInstance"));
                Assert.That(exception.Message, Does.Contain("Pending command: post"));
                Assert.That(exception.Message, Does.Contain("Message state: sent"));
                Assert.That(exception.Message, Does.Contain("Channel backlog:"));
                Assert.That(exception.Message, Does.Contain(nameof(StalledCommandTimesOutWithDiagnostics)));
            });
        }
        finally
        {
            release.Set();
        }

        await completed.Task.WaitAsync(TimeSpan.FromSeconds(10));
    }
}
