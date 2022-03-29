/* istanbul ignore file */
import { Box, Card, Grid, GridProps, Paper, styled, Typography } from '@mui/material';
import { Dispenser, FleetState, Ingestor, RobotState, TaskState } from 'api-client';
import { AxiosResponse } from 'axios';
import React from 'react';
import { PaginationOptions, RobotInfo, RobotTable, RobotTableData } from 'react-components';
import { Map, MapProps } from 'react-leaflet';
import {
  useDispenserStatesRef,
  useFleetStateRef,
  useIngestorStatesRef,
} from '../../util/common-subscriptions';
import { BuildingMapContext, RmfIngress, RmfIngressContext } from '../rmf-app';
import ScheduleVisualizer from '../schedule-visualizer';

const MemoRobotInfo = React.memo(RobotInfo);

const UpdateRate = 1000;
const prefix = 'robot-page';
const classes = {
  container: `${prefix}-container`,
  robotPanel: `${prefix}-robot-panel`,
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
  [`& .${classes.robotPanel}`]: {
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

function getTaskProgress(robot: RobotState, task?: TaskState) {
  if (
    !robot.task_id ||
    !robot.unix_millis_time ||
    !task ||
    !task.unix_millis_start_time ||
    !task.estimate_millis
  ) {
    return undefined;
  }
  return Math.min(
    (robot.unix_millis_time - task.unix_millis_start_time) /
      (task.estimate_millis - task.unix_millis_start_time),
    1,
  );
}

function NoSelectedRobot() {
  return (
    <Box sx={{ display: 'flex', height: '100%', alignItems: 'center', justifyContent: 'center' }}>
      <Typography variant="h6" align="center">
        Click on a robot to view more information
      </Typography>
    </Box>
  );
}

export function RobotPage() {
  const rmfIngress = React.useContext(RmfIngressContext);
  const sioClient = React.useContext(RmfIngressContext)?.sioClient;
  const buildingMap = React.useContext(BuildingMapContext);
  const [leafletMap, setLeafletMap] = React.useState<Map<MapProps, L.Map>>();
  // FIXME: RobotTable does not know about which fleet a robot belongs to, so there is no way to
  // distinguish between robots with the same name in different fleets, events fired by RobotTable,
  // will be broken in that case.
  const robotStatesRef = React.useRef<Record<string, RobotState>>({});
  const taskStatesRef = React.useRef<Record<string, TaskState>>({});
  const [robotTableData, setRobotTableData] = React.useState<RobotTableData[]>([]);
  const [selectedRobot, setSelectedRobot] = React.useState<RobotState | undefined>(undefined);
  const [selectedTask, setSelectedTask] = React.useState<TaskState | undefined>(undefined);
  const [page, setPage] = React.useState(0);

  const [_triggerRender, setTriggerRender] = React.useState(0); // eslint-disable-line @typescript-eslint/no-unused-vars
  React.useEffect(() => {
    const interval = setInterval(() => setTriggerRender((prev) => prev + 1), UpdateRate);
    return () => clearInterval(interval);
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
      const newRobotTableData: RobotTableData[] = [];
      const newRobotStates: Record<string, RobotState> = {};
      fleets.forEach((fleet) => {
        if (!fleet.robots) {
          return;
        }
        Object.values(fleet.robots).forEach((robot) => {
          if (!robot.name) {
            return;
          }
          const activeTask =
            robot.task_id != null && tasks[robot.task_id] ? tasks[robot.task_id] : undefined;
          newRobotTableData.push({
            name: robot.name,
            battery: robot.battery,
            status: robot.status,
            estFinishTime: activeTask && activeTask.estimate_millis,
          });
          newRobotStates[robot.name] = robot;
          robotStatesRef.current[robot.name] = robot;
          if (selectedRobot && selectedRobot.name && robot.name) {
            setSelectedRobot(robot);
            if (robot.task_id) {
              setSelectedTask(taskStatesRef.current[robot.task_id]);
            }
          }
        });
      });
      setRobotTableData(newRobotTableData);
      taskStatesRef.current = tasks;
      setDispensers(dispensers);
      setIngestors(ingestors);
      setFleets(fleets);
    })();
  }, [rmfIngress, _triggerRender, selectedRobot]);

  const handleRobotClick = async (
    _ev: React.MouseEvent<HTMLDivElement, MouseEvent>,
    robotName: string,
  ) => {
    const robot = robotStatesRef.current[robotName];
    console.log(robotName, robot);
    if (!robot) {
      return;
    }
    setSelectedRobot(robot);
    if (robot.task_id) {
      setSelectedTask(taskStatesRef.current[robot.task_id]);
    }

    // zoom to robot
    leafletMap &&
      leafletMap.leafletElement.setView(
        [robot.location ? robot.location.y : 0.0, robot.location ? robot.location.x : 0.0],
        5.5,
        {
          animate: true,
        },
      );
  };

  const paginationOptions = React.useMemo<PaginationOptions>(
    () => ({
      count: robotTableData.length,
      page,
      onPageChange: (_, page) => setPage(page),
      rowsPerPage: 10,
      rowsPerPageOptions: [10],
    }),
    [page, robotTableData.length],
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
      <Grid item xs={8}>
        <RobotTable
          className={classes.robotTable}
          robots={robotTableData}
          paginationOptions={paginationOptions}
          onRobotClick={handleRobotClick}
        />
      </Grid>
      <Grid item xs={4}>
        <Paper variant="outlined" className={classes.detailPanelContainer}>
          {selectedRobot && selectedRobot.name ? (
            <MemoRobotInfo
              robotName={selectedRobot.name}
              assignedTask={selectedRobot.task_id}
              battery={selectedRobot.battery}
              estFinishTime={selectedTask && selectedTask.estimate_millis}
              taskProgress={getTaskProgress(selectedRobot, selectedTask)}
              taskStatus={selectedTask?.status}
              updateTime={selectedRobot.unix_millis_time}
            />
          ) : (
            <NoSelectedRobot />
          )}
        </Paper>
      </Grid>
    </StyledGrid>
  );
}
