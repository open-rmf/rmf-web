import React from 'react';
import ReactDOM from 'react-dom';
import LiftsPanel from '../components/lifts-panel';
import buildingMap from './data/building-map';
import liftStates from './data/lift-states';
import { FakeTransport } from './fake-transport';

(async () => {
  const bm = await buildingMap();
  const transport = new FakeTransport();

  ReactDOM.render(
    <LiftsPanel
      transport={transport}
      lifts={bm.lifts}
      liftStates={liftStates}
    />,
    document.getElementById('root'),
  );

})();
