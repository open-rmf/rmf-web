import React from 'react';
import { makeStyles, Grid } from '@material-ui/core';
import { BuildingMap } from 'api-client';
import { getPlaces } from 'react-components';
import { AppBar } from './components/appbar';
import { TaskForm } from './components/task-form';
import { sioClient } from './app-config';
import './App.css';

const useStyles = makeStyles((theme) => ({
  root: {
    padding: theme.spacing(4),
  },
}));

function App() {
  const classes = useStyles();
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
    <div>
      <AppBar />
      <Grid container>
        <Grid item xs={6} className={classes.root}>
          <TaskForm placeNames={placeNames} />
        </Grid>
        <Grid item>display</Grid>
      </Grid>
    </div>
  );
}

export default App;
