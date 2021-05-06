const allPackages = [
  'packages/react-components',
  'packages/rmf-auth',
  'packages/reporting-server',
  'packages/reporting',
  'packages/dashboard',
  'packages/api-server',
  'packages/api-client',
  'packages/rmf-models',
  'packages/ros-translator',
];

// hardcoded for now
const packageDeps = {
  'packages/dashboard': [
    'packages/react-components',
    'packages/api-client',
    'packages/rmf-auth',
    'packages/rmf-models',
  ],
  'packages/react-components': ['packages/api-client', 'packages/rmf-models'],
  'packages/reporting': [
    'packages/react-components',
    'packages/rmf-auth',
    'packages/reporting-server',
  ],
  'packages/api-client': ['packages/rmf-models'],
};

function getPackageDeps(pkg) {
  const recur = (pkg, cur) => {
    if (packageDeps[pkg]) {
      packageDeps[pkg].forEach((p) => {
        recur(p, cur);
      });
    }
    cur.add(pkg);
    return cur;
  };
  return recur(pkg, new Set());
}

module.exports = {
  allPackages,
  getPackageDeps,
};
