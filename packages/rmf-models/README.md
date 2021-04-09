# Generating models

This most of this package contains generated code, simply run `generate-models.sh` to generate an updated version of the models.

## Regarding side effects of pinning rmf-model version

The advantage of pinning the version of rmf and rmf-server is that it releases the dependency of those packages. With pinning, there is no longer a need to have rmf sourced to build `rmf-models`, this allows dependents of `rmf-models` to free themselve of the transitive dependeny to rmf, this is especially for frontend only packages like `react-components` so that one can start contributing without all the hassle of building rmf.

However, the biggest downside of pinning the version is that it makes it much harder to develop `rmf-models` dependencies concurrently with `rmf-models` itself. The reason is because we must never allow a PR of `rmf-models` that generates from anything other than the `main` branch. Because open-rmf use squash merges by default, any versions not from the `main` branch pinned to `rmf-models` will essentially be lost once the PR gets merged.

Keep in mind that this is also true for `rmf-web` as well, so you CAN'T submit a PR that updates both `rmf-server` and `rmf-models`, as the version will be lost when that get merged. The `rmf-server` needs to be merged into `main` first before the PR for `rmf-models` can be submitted.
