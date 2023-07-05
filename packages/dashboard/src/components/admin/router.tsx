import { Route, Routes, Link, Navigate, useMatch } from 'react-router-dom';
import { AdminDrawer } from './drawer';
import { RoleListPage } from './role-list-page';
import { UserListPage } from './user-list-page';
import { UserProfilePage } from './user-profile-page';

export function AdminRouter(): JSX.Element {
  const match = useMatch('/admin');
  console.log(match);

  return (
    <>
      <Routes>
        <Route path="/admin/*" element={<Navigate to="/admin" />} />
        {/* <Route path={':user'} element={<UserProfilePage />} /> */}
        <Route path={'/admin/users'} element={<UserListPage />} />
        {/* <Route path={'roles'}>
          <RoleListPage />
        </Route>
        <Route>
          <Link to={"users"} />
        </Route> */}
      </Routes>
      <AdminDrawer />
    </>
  );
}
