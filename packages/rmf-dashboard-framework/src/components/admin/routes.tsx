import { Route } from 'react-router-dom';

import { AdminDrawer } from './drawer';
import { RoleListPage } from './role-list-page';
import { UserListPage } from './user-list-page';
import { UserProfilePage } from './user-profile-page';

export const adminRoutes = (
  <Route path="admin" element={<AdminDrawer />}>
    <Route path={'users/:username'} element={<UserProfilePage />} />
    <Route path={'users'} element={<UserListPage />} />
    <Route path={'roles'} element={<RoleListPage />} />
  </Route>
);
