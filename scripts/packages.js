const allPackages = [
  'packages/ros2-bridge',
  'packages/react-components',
  'packages/rmf-auth',
  'packages/reporting',
  'packages/dashboard',
  'packages/api-server',
  'packages/api-client',
  'packages/rmf-models',
  'packages/ros_translator',
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
  'packages/reporting': ['packages/react-components', 'packages/rmf-auth'],
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
