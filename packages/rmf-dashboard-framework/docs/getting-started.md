## Getting Started

Welcome to RMF Dashboard Framework! This tutorial will guide you through the initial setup and provide a basic example to help you start building your dashboard application.

### Setup

Prerequisites:
* pnpm
* Latest Nodejs LTS

Follow https://pnpm.io/installation to install pnpm, you can then install nodejs using pnpm

```bash
pnpm env use --global lts
```

Clone this repo and install deps

```bash
pnpm create vite -t react-ts basic-dashboard
cd basic-dashboard
# TODO(koonpeng): install from npmjs after package is published.
git clone --depth 1 https://github.com/open-rmf/rmf-web
```

<!-- TODO(koonpeng): not needed after package is published -->
Add modify the following to resolve `rmf-dashboard-framework` as if it is installed from npmjs.

pnpm-workspace.yaml (create if not exist)
```yaml
packages:
  - "."
  - "rmf-web/**/*"
```

package.json
```
"dependencies": {
  ...
  "rmf-dashboard-framework": "workspace:*"
}
```

tsconfig.app.json
```
"paths": {
  "rmf-dashboard-framework/*": ["./node_modules/rmf-dashboard-framework/src/*"]
}
```

Finally install and build the deps

```bash
# TODO(koonpeng): -w flag not needed after rmf-dashboard-framework is published
pnpm add -w @fontsource/roboto
pnpm install --filter basic-dashboard...
pnpm --filter basic-dashboard^... build
```

### Creating a Basic Dashboard

At this point, you should be able to follow the [demo dashboard example](../examples/demo/main.tsx).
