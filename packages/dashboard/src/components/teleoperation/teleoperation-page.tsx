/* istanbul ignore file */
import { Box, Card, Grid, GridProps, Paper, styled, Typography } from '@mui/material';
import { Dispenser, FleetState, Ingestor, RobotState, TaskState } from 'api-client';
import { AxiosResponse } from 'axios';
import React from 'react';
import {
  PaginationOptions,
  TeleoperationInfo,
  TeleoperationRobotTable,
  TeleoperationRobotConfig,
} from 'react-components';
import { Map, MapProps } from 'react-leaflet';
import { BasePath } from '../../util/url';
import {
  useDispenserStatesRef,
  useFleetStateRef,
  useIngestorStatesRef,
} from '../../util/common-subscriptions';
import { BuildingMapContext, RmfIngress, RmfIngressContext } from '../rmf-app';
import ScheduleVisualizer from '../schedule-visualizer';
import { parse } from 'yaml';

const MemoTeleoperationInfo = React.memo(TeleoperationInfo);

const UpdateRate = 1000;
const prefix = 'teleoperation-page';
const classes = {
  container: `${prefix}-container`,
  teleoperationPanel: `${prefix}-teleoperation-panel`,
  mapPanel: `${prefix}-map-panel`,
  detailPanelContainer: `${prefix}-detail-container`,
  robotTable: `${prefix}-robot-table`,
};
const StyledGrid = styled((props: GridProps) => <Grid {...props} />)(({ theme }) => ({
  [`&.${classes.container}`]: {
    padding: `${theme.spacing(4)}`,
    height: '100%',
    backgroundColor: theme.palette.background.default,
  },
  [`& .${classes.teleoperationPanel}`]: {
    height: '100%',
  },
  [`& .${classes.mapPanel}`]: {
    height: '100%',
    flex: '1 0 auto',
  },
  [`& .${classes.detailPanelContainer}`]: {
    padding: theme.spacing(2),
    boxSizing: 'border-box',
    height: '100%',
  },
  [`& .${classes.robotTable}`]: {
    height: '100%',
    display: 'flex',
    flexDirection: 'column',
  },
}));

async function fetchActiveTaskStates(fleets: FleetState[], rmfIngress: RmfIngress) {
  const taskIds = fleets.flatMap(
    (fleet) => fleet.robots && Object.values(fleet.robots).map((robot) => robot.task_id),
  );
  const promises: Promise<AxiosResponse<TaskState>>[] = [];
  for (const taskId of taskIds) {
    if (taskId) {
      promises.push(rmfIngress.tasksApi.getTaskStateTasksTaskIdStateGet(taskId));
    }
  }
  const results = await Promise.all(promises);
  return results.reduce<Record<string, TaskState>>((acc, result) => {
    acc[result.data.booking.id] = result.data;
    return acc;
  }, {});
}

function NoSelectedRobot() {
  return (
    <Box sx={{ display: 'flex', height: '100%', alignItems: 'center', justifyContent: 'center' }}>
      <Typography variant="h6" align="center">
        Click on a robot to access teleoperation controls
      </Typography>
    </Box>
  );
}

export function TeleoperationPage() {
  const rmfIngress = React.useContext(RmfIngressContext);
  const sioClient = React.useContext(RmfIngressContext)?.sioClient;
  const buildingMap = React.useContext(BuildingMapContext);
  const [leafletMap, setLeafletMap] = React.useState<Map<MapProps, L.Map>>();
  // FIXME: RobotTable does not know about which fleet a robot belongs to, so there is no way to
  // distinguish between robots with the same name in different fleets, events fired by RobotTable,
  // will be broken in that case.
  const robotStatesRef = React.useRef<Record<string, RobotState>>({});
  const taskStatesRef = React.useRef<Record<string, TaskState>>({});
  const [robotsConfig, setRobotsConfig] = React.useState<TeleoperationRobotConfig[]>([]);
  const [selectedRobotConfig, setSelectedRobotConfig] = React.useState<
    TeleoperationRobotConfig | undefined
  >(undefined);
  const [companyId, setCompanyId] = React.useState<string>('');
  const [page, setPage] = React.useState(0);

  const [_triggerRender, setTriggerRender] = React.useState(0); // eslint-disable-line @typescript-eslint/no-unused-vars
  React.useEffect(() => {
    const interval = setInterval(() => setTriggerRender((prev) => prev + 1), UpdateRate);
    return () => clearInterval(interval);
  }, []);

  const parseRobotManagerConfig = async () => {
    const newRobotConfigs: TeleoperationRobotConfig[] = [];
    const TELEOP_CONFIG_PATH = process.env.REACT_APP_TELEOP_CONFIG;
    if (!TELEOP_CONFIG_PATH) {
      console.log('No config path for teleoperation and video streaming found.');
      setCompanyId('');
      setRobotsConfig([]);
      return;
    }

    const response = await fetch(`${TELEOP_CONFIG_PATH}`);
    const text = await response.text();
    const config = parse(text);
    for (let key in config) {
      if (key === 'robots') {
        for (let robot in config[key]) {
          newRobotConfigs.push({
            name: robot,
            id: config[key][robot].robot_config.robot_id,
            accessKey: config[key][robot].robot_config.access_key,
          });
        }
      }
    }

    setCompanyId(config.robotmanager.company_id);
    setRobotsConfig(newRobotConfigs);
  };

  React.useEffect(() => {
    parseRobotManagerConfig();
  }, []);

  // get work cells to display on map
  const [dispensers, setDispensers] = React.useState<Dispenser[]>([]);
  useDispenserStatesRef(sioClient, dispensers);
  const [ingestors, setIngestors] = React.useState<Ingestor[]>([]);
  useIngestorStatesRef(sioClient, ingestors);

  // schedule visualizer fleet
  const [fleets, setFleets] = React.useState<FleetState[]>([]);
  const fleetStatesRef = useFleetStateRef(sioClient, fleets);

  // fetch data
  React.useEffect(() => {
    if (!rmfIngress) {
      return;
    }
    (async () => {
      const dispensers = (await rmfIngress.dispensersApi.getDispensersDispensersGet()).data;
      const ingestors = (await rmfIngress.ingestorsApi.getIngestorsIngestorsGet()).data;
      const fleets = (await rmfIngress.fleetsApi.getFleetsFleetsGet()).data;
      const tasks = await fetchActiveTaskStates(fleets, rmfIngress);
      const newRobotStates: Record<string, RobotState> = {};
      fleets.forEach((fleet) => {
        if (!fleet.robots) {
          return;
        }
        Object.values(fleet.robots).forEach((robot) => {
          if (!robot.name) {
            return;
          }
          newRobotStates[robot.name] = robot;
          robotStatesRef.current[robot.name] = robot;
        });
      });
      taskStatesRef.current = tasks;
      setDispensers(dispensers);
      setIngestors(ingestors);
      setFleets(fleets);
    })();
  }, [rmfIngress]);

  const handleRobotClick = async (
    _ev: React.MouseEvent<HTMLDivElement, MouseEvent>,
    robotName: string,
  ) => {
    for (const config of robotsConfig) {
      if (config.name === robotName) {
        setSelectedRobotConfig(config);
      }
    }
  };

  const paginationOptions = React.useMemo<PaginationOptions>(
    () => ({
      count: robotsConfig.length,
      page,
      onPageChange: (_, page) => setPage(page),
      rowsPerPage: 10,
      rowsPerPageOptions: [10],
    }),
    [page, robotsConfig.length],
  );

  return (
    <StyledGrid container className={classes.container} wrap="nowrap" spacing={2}>
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
      <Grid item xs={4}>
        <TeleoperationRobotTable
          className={classes.robotTable}
          robots={robotsConfig}
          paginationOptions={paginationOptions}
          onRobotClick={handleRobotClick}
        />
      </Grid>
      <Grid item xs={8}>
        <Paper variant="outlined" className={classes.detailPanelContainer}>
          {selectedRobotConfig ? (
            <MemoTeleoperationInfo
              name={selectedRobotConfig.name}
              id={selectedRobotConfig.id}
              accessKey={selectedRobotConfig.accessKey}
              companyId={companyId}
            />
          ) : (
            <NoSelectedRobot />
          )}
        </Paper>
      </Grid>
    </StyledGrid>
  );
}
