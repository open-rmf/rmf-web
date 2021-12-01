import React from 'react';
import { Grid, styled } from '@mui/material';
import { BuildingMap, Task, TaskSummary } from 'api-client';
import { getPlaces } from 'react-components';
import { AppBar } from './components/appbar';
import { TaskForm } from './components/task-form';
import { TaskDisplay } from './components/task-display';
import { sioClient, taskApi } from './app-config';
import './App.css';

const classes = {
  root: 'minimal-app',
};

const StyledDiv = styled('div')(({ theme }) => ({
  [`& .${classes.root}`]: {
    padding: theme.spacing(4),
  },
}));

function App() {
  const [buildingMap, setBuildingMap] = React.useState<BuildingMap | null>(null);
  const [placeNames, setPlaceNames] = React.useState<string[]>([]);
  const [fetchedTasks, setFetchedTasks] = React.useState<Task[]>([]);
  const [updatedSummaries, setUpdatedSummaries] = React.useState<Record<string, TaskSummary>>({});
  const tasks = React.useMemo(
    () => fetchedTasks.map((t) => ({ ...t, summary: updatedSummaries[t.task_id] || t.summary })),
    [fetchedTasks, updatedSummaries],
  );

  const fetchTasks = React.useCallback(async () => {
    if (!taskApi) {
      return [];
    }
    const resp = await taskApi.getTasksTasksGet(
      undefined,
      undefined,
      undefined,
      undefined,
      undefined,
      undefined,
      undefined,
      undefined,
      undefined,
      undefined,
      undefined,
      '-priority,-start_time',
    );
    const results = resp.data as Task[];
    setFetchedTasks(results);
  }, []);

  React.useEffect(() => {
    (async () => {
      await fetchTasks();
    })();
  });

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
    if (!sioClient) return;
    const subs = fetchedTasks.map((t) =>
      sioClient.subscribeTaskSummary(t.task_id, (newSummary) =>
        setUpdatedSummaries((prev) => ({
          ...prev,
          [newSummary.task_id]: newSummary,
        })),
      ),
    );
    return () => {
      subs.forEach((s) => sioClient.unsubscribe(s));
    };
  }, [fetchedTasks]);

  React.useEffect(() => {
    if (!buildingMap) {
      return;
    }
    const places = getPlaces(buildingMap);
    const placeNames = places.map((p) => p.vertex.name);
    setPlaceNames(placeNames);
  }, [buildingMap]);

  return (
    <StyledDiv>
      <AppBar />
      <Grid direction="row" container className={classes.root} spacing={3}>
        <Grid item xs={4}>
          <TaskForm placeNames={placeNames} onFetchTask={fetchTasks} />
        </Grid>
        <Grid item xs={8}>
          <TaskDisplay tasks={tasks.map((t) => t.summary)} />
        </Grid>
      </Grid>
    </StyledDiv>
  );
}

export default App;
