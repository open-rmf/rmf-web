import React from 'react';
import { BuildingMap } from 'api-client';
import { getPlaces } from 'react-components';
import { AppBar } from './components/appbar';
import { sioClient } from './app-config';
import './App.css';

function App() {
  const [buildingMap, setBuildingMap] = React.useState<BuildingMap | null>(null);
  const [placeNames, setPlaceNames] = React.useState<string[]>([]);
  React.useEffect(() => {
    if (!sioClient) {
      return;
    }
    const sub = sioClient.subscribeBuildingMap(setBuildingMap);
    return () => {
      sioClient.unsubscribe(sub);
    };
  }, []);

  React.useEffect(() => {
    if (!buildingMap) {
      return;
    }
    const places = getPlaces(buildingMap);
    const placeNames = places.map((p) => p.vertex.name);
    setPlaceNames(placeNames);
  }, [buildingMap]);

  return (
    <div className="App">
      <AppBar />
    </div>
  );
}

export default App;
