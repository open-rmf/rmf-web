/* istanbul ignore file */

import React from 'react';
import { RmfIngressContext } from '../rmf-app';
import { usePageStyles } from './page-css';
import { RoleListCard } from './role-list-card';

export function RoleListPage(): JSX.Element {
  const classes = usePageStyles();
  const rmfIngress = React.useContext(RmfIngressContext);
  const adminApi = rmfIngress?.adminApi;
  const [roles, setRoles] = React.useState<string[]>([]);

  React.useEffect(() => {
    if (!adminApi) return;
    let cancel = false;
    (async () => {
      const results = await adminApi.getRolesAdminRolesGet();
      if (cancel || results.status !== 200) return;
      setRoles(results.data);
    })();
    return () => {
      cancel = true;
    };
  }, [adminApi]);

  return (
    <div className={classes.pageRoot}>
      <RoleListCard roles={roles} />
    </div>
  );
}
