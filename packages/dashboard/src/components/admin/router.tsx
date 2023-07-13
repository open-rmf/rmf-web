import { Route, Routes, Navigate, Outlet } from 'react-router-dom';
import { AdminDrawer } from './drawer';
import { RoleListPage } from './role-list-page';
import { UserListPage } from './user-list-page';
import { UserProfilePage } from './user-profile-page';

export function AdminRouter(): JSX.Element {
  return (
    <>
      <AdminDrawer />
      <Routes>
        <Route path={'/*'} element={<Navigate to={'users'} />} />
        <Route path={'/users/:username'} element={<UserProfilePage />} />
        <Route path={'users'} element={<UserListPage />} />
        <Route path={'roles'} element={<RoleListPage />} />
        <Route element={<Navigate to={'users'} />} />
      </Routes>
      <Outlet />
    </>
  );
}
