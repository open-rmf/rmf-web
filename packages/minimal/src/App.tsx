import React from 'react';
import { Grid, styled } from '@mui/material';
import { BuildingMap, Task, TaskSummary } from 'api-client';
import { getPlaces } from 'react-components';
import { LoginPage, PrivateRoute } from 'rmf-auth';
import { BrowserRouter, Redirect, Route, Switch } from 'react-router-dom';
import { AppBar } from './components/appbar';
import { TaskForm } from './components/task-form';
import { TaskDisplay } from './components/task-display';
import { sioClient, taskApi, authenticator } from './app-config';
import { LoginRoute, DashboardRoute } from './utils/url';
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
  const [authInitialized, setAuthInitialized] = React.useState(!!authenticator.user);
  const [user, setUser] = React.useState<string | null>(authenticator.user || null);
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
    let cancel = false;
    const onUserChanged = (newUser: string | null) => setUser(newUser);
    authenticator.on('userChanged', onUserChanged);
    (async () => {
      await authenticator.init();
      if (cancel) {
        return;
      }
      setUser(authenticator.user || null);
      console.log('setting authenticator');
      setAuthInitialized(true);
    })();
    return () => {
      cancel = true;
      authenticator.off('userChanged', onUserChanged);
    };
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

  const loginRedirect = React.useMemo(() => <Redirect to={LoginRoute} />, []);
  return authInitialized ? (
    <BrowserRouter>
      {user ? (
        <Switch>
          <Route exact path={LoginRoute}>
            <Redirect to={DashboardRoute} />
          </Route>
          <PrivateRoute
            exact
            path={DashboardRoute}
            unauthorizedComponent={loginRedirect}
            user={user}
          >
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
          </PrivateRoute>
          <PrivateRoute unauthorizedComponent={loginRedirect} user={user}>
            <Redirect to={DashboardRoute} />
          </PrivateRoute>
        </Switch>
      ) : (
        <Switch>
          <Route exact path={LoginRoute}>
            <LoginPage
              title={'Minimal App'}
              logo="assets/ros-health.png"
              onLoginClick={() => authenticator.login(`${window.location.origin}${DashboardRoute}`)}
            />
          </Route>
          <Route>
            <Redirect to={LoginRoute} />
          </Route>
        </Switch>
      )}
    </BrowserRouter>
  ) : null;
}

export default App;
