import React from 'react';
import { DataConfigContext } from '../app-contexts';
import LoopTaskPage from '../single-responsibility/send-loop-task';

export default function Dashboard(_props: {}): React.ReactElement {
  const data = React.useContext(DataConfigContext);

  return (
    <div>
      <LoopTaskPage data={data} />
    </div>
  );
}
