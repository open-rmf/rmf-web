import React from 'react';
import { DataConfigContext } from '../app-contexts';
import { PlacesContext } from '../rmf-app/contexts';
import { UserContext } from '../auth/contexts';
import LoopTaskPage from '../single-responsibility/send-loop-task';

export default function Dashboard(_props: {}): React.ReactElement {
  const data = React.useContext(DataConfigContext);
  const user = React.useContext(UserContext);

  const places = React.useContext(PlacesContext);
  const placeNames = places.map((p) => p.vertex.name);

  return (
    <div>
      <LoopTaskPage data={data} places={placeNames} user={user} />
    </div>
  );
}
