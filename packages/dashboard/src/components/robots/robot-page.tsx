/* istanbul ignore file */

import { styled, GridProps, Grid, Card } from '@mui/material';
import React from 'react';
import { Fleet } from 'api-client';
import { MapProps, Map } from 'react-leaflet';
import { RobotPanel, VerboseRobot } from 'react-components';
import {
  BuildingMapContext,
  RmfIngressContext,
  DispensersContext,
  IngestorsContext,
} from '../rmf-app';
import ScheduleVisualizer from '../schedule-visualizer';
import {
  useFleets,
  useFleetStateRef,
  useDispenserStatesRef,
  useIngestorStatesRef,
} from '../../util/common-subscriptions';

const UpdateRate = 1000;
const prefix = 'robot-page';
const classes = {
  container: `${prefix}-container`,
  robotPanel: `${prefix}-robot-panel`,
  mapPanel: `${prefix}-map-panel`,
};
const StyledGrid = styled((props: GridProps) => <Grid {...props} />)(({ theme }) => ({
  [`&.${classes.container}`]: {
    padding: `${theme.spacing(4)}`,
    height: '100%',
    backgroundColor: theme.palette.background.default,
  },
  [`& .${classes.robotPanel}`]: {
    height: '100%',
  },
  [`& .${classes.mapPanel}`]: {
    height: '100%',
    marginRight: theme.spacing(2),
    flex: '1 0 auto',
  },
}));

export function RobotPage() {
  const rmfIngress = React.useContext(RmfIngressContext);
  const sioClient = React.useContext(RmfIngressContext)?.sioClient;
  const buildingMap = React.useContext(BuildingMapContext);
  const [leafletMap, setLeafletMap] = React.useState<Map<MapProps, L.Map>>();

  const [_triggerRender, setTriggerRender] = React.useState(0); // eslint-disable-line @typescript-eslint/no-unused-vars
  React.useEffect(() => {
    const interval = setInterval(() => setTriggerRender((prev) => prev + 1), UpdateRate);
    return () => clearInterval(interval);
  }, []);

  // get work cells to display on map
  const dispensers = React.useContext(DispensersContext);
  useDispenserStatesRef(sioClient, dispensers);

  const ingestors = React.useContext(IngestorsContext);
  useIngestorStatesRef(sioClient, ingestors);

  // schedule visualizer fleet
  const [fleets, setFleets] = React.useState<Fleet[]>([]);
  useFleets(rmfIngress, setFleets);
  const fleetStatesRef = useFleetStateRef(sioClient, fleets);

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

  const onRobotZoom = (robot: VerboseRobot) => {
    leafletMap &&
      leafletMap.leafletElement.setView([robot.state.location.y, robot.state.location.x], 5.5, {
        animate: true,
      });
  };

  React.useEffect(() => {
    fetchVerboseRobots();
  }, [fetchVerboseRobots]);

  return (
    <StyledGrid container className={classes.container}>
      <Grid item xs={4}>
        <Card variant="outlined" className={classes.mapPanel}>
          {buildingMap && (
            <ScheduleVisualizer
              buildingMap={buildingMap}
              dispensers={dispensers}
              ingestors={ingestors}
              fleetStates={Object.assign({}, fleetStatesRef.current)}
              mode="normal"
              zoom={4.5}
              ref={(map: Map<MapProps, L.Map>) => setLeafletMap(map)}
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
            onPageChange: (_ev, newPage) => setPage(newPage),
          }}
          verboseRobots={verboseRobots}
          onRobotZoom={onRobotZoom}
        />
      </Grid>
    </StyledGrid>
  );
}
