Tests are indirectly done in https://github.com/osrf/romi-dashboard.

It's hard to write tests for this, the most feasible approach is to mock the trajectory server but that is flaky at best and is not worth the effort.

Instead, we should focus on integration/e2e tests, but doing so requires depdency on many components of rmf, the main challenge is that it is not easy to build and launch all those components, instead of replicating the work done by romi-dashboard, it is easier to delegate the tests there.
