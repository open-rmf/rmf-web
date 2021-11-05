/* istanbul ignore file */

import { makeStyles, Grid, Card } from '@material-ui/core';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { Fleet } from 'api-client';
import { RobotPanel, VerboseRobot } from 'react-components';
import { LeafletContext } from 'react-leaflet';
import {
  BuildingMapContext,
  RmfIngressContext,
  DispensersContext,
  IngestorsContext,
} from '../rmf-app';
import ScheduleVisualizer from '../schedule-visualizer';
import {
  GetFleets,
  SubscribeFleet,
  SubscribeDispenser,
  SubscribeIngestor,
} from '../../util/common-subscriptions';

const UpdateRate = 1000;

const useStyles = makeStyles((theme) => ({
  robotPanel: {
    height: '100%',
  },
  container: {
    padding: `${theme.spacing(4)}px`,
    height: '100%',
    backgroundColor: theme.palette.background.default,
  },
  mapPanel: {
    height: '100%',
    marginRight: theme.spacing(2),
    flex: '1 0 auto',
  },
}));

export function RobotPage() {
  const classes = useStyles();
  const rmfIngress = React.useContext(RmfIngressContext);
  const sioClient = React.useContext(RmfIngressContext)?.sioClient;
  const buildingMap = React.useContext(BuildingMapContext);

  const [leafletMap, setLeafletMap] = React.useState<LeafletContext>({});
  const [_triggerRender, setTriggerRender] = React.useState(0); // eslint-disable-line @typescript-eslint/no-unused-vars
  React.useEffect(() => {
    const interval = setInterval(() => setTriggerRender((prev) => prev + 1), UpdateRate);
    return () => clearInterval(interval);
  }, []);

  // get work cells to display on map
  const dispensers = React.useContext(DispensersContext);
  const dispenserStatesRef = React.useRef<Record<string, RmfModels.DispenserState>>({});
  SubscribeDispenser(sioClient, dispensers, dispenserStatesRef);

  const ingestors = React.useContext(IngestorsContext);
  const ingestorStatesRef = React.useRef<Record<string, RmfModels.IngestorState>>({});
  SubscribeIngestor(sioClient, ingestors, ingestorStatesRef);

  // schedule visualizer fleet
  const [fleets, setFleets] = React.useState<Fleet[]>([]);
  GetFleets(rmfIngress, setFleets);
  const fleetStatesRef = React.useRef<Record<string, RmfModels.FleetState>>({});
  SubscribeFleet(sioClient, fleets, fleetStatesRef);

  // robot panel stuff
  const [hasMore, setHasMore] = React.useState(true);
  const [page, setPage] = React.useState(0);
  const [verboseRobots, setVerboseRobots] = React.useState<VerboseRobot[]>([]);
  const fetchVerboseRobots = React.useCallback(async () => {
    if (!rmfIngress) {
      setHasMore(false);
      return [];
    }
    const resp = await rmfIngress.fleetsApi.getRobotsFleetsRobotsGet(
      undefined,
      undefined,
      11,
      page * 10,
      'fleet_name,robot_name',
    );
    const robots = resp.data as VerboseRobot[];
    setHasMore(robots.length > 10);
    const slicedRobots = robots.slice(0, 10);
    setVerboseRobots(slicedRobots);
    return slicedRobots;
  }, [rmfIngress, page]);

  React.useEffect(() => {
    fetchVerboseRobots();
  }, [fetchVerboseRobots]);

  return (
    <Grid container className={classes.container}>
      <Grid item xs={4}>
        <Card variant="outlined" className={classes.mapPanel}>
          {buildingMap && (
            <ScheduleVisualizer
              buildingMap={buildingMap}
              dispensers={dispensers}
              ingestors={ingestors}
              fleetStates={Object.assign({}, fleetStatesRef.current)}
              mode="normal"
              setLeafletMap={setLeafletMap}
              zoom={4.5}
            />
          )}
        </Card>
      </Grid>
      <Grid item xs={8}>
        <RobotPanel
          className={classes.robotPanel}
          fetchVerboseRobots={fetchVerboseRobots}
          paginationOptions={{
            count: hasMore ? -1 : page * 10 + verboseRobots.length,
            rowsPerPage: 10,
            rowsPerPageOptions: [10],
            page,
            onChangePage: (_ev, newPage) => setPage(newPage),
          }}
          verboseRobots={verboseRobots}
          leafletMap={leafletMap}
        />
      </Grid>
    </Grid>
  );
}
