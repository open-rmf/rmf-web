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
pnpm add https://github.com/open-rmf/rmf-web#koonpeng/merge-react-components:packages/rmf-dashboard-framework
pnpm install
```

### Creating a Basic Dashboard


