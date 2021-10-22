/* istanbul ignore file */

import { Grid, IconButton, makeStyles } from '@material-ui/core';
import ViewListIcon from '@material-ui/icons/ViewList';
import ViewModuleIcon from '@material-ui/icons/ViewModule';
import React from 'react';
import { DoorCell as DoorCell_, DoorData, DoorTable, useAsync, Window } from 'react-components';
import * as RmfModels from 'rmf-models';
import { throttleTime } from 'rxjs';
import { RmfIngressContext, RxRmfContext } from '../rmf-app';
import { allWindows, ManagedWindowProps, WindowClass } from '../window';

const DoorCell = React.memo(DoorCell_);

export const DoorsWindow: React.FC<ManagedWindowProps> = React.forwardRef(
  ({ children, ...otherProps }, ref) => {
    const safeAsync = useAsync();
    const rmfIngress = React.useContext(RmfIngressContext);
    const rxRmf = React.useContext(RxRmfContext);
    const [doors, setDoors] = React.useState<DoorData[]>([]);
    const [doorStates, setDoorStates] = React.useState<Record<string, RmfModels.DoorState>>({});
    const [isCellView, setIsCellView] = React.useState(true);

    React.useEffect(() => {
      if (!rmfIngress) return;
      (async () => {
        const buildingMap = (await safeAsync(rmfIngress.buildingApi.getBuildingMapBuildingMapGet()))
          .data as RmfModels.BuildingMap;
        const doors = buildingMap.levels.flatMap((level) =>
          level.doors.map((door) => ({ level: level.name, door } as DoorData)),
        );
        setDoors(doors);
      })();
    }, [safeAsync, rmfIngress]);

    React.useEffect(() => {
      if (!rxRmf) return;
      const subs = doors.map((door) =>
        rxRmf
          .doorStates(door.door.name)
          .pipe(throttleTime(1000))
          .subscribe(
            (doorState) =>
              doorState && setDoorStates((prev) => ({ ...prev, [door.door.name]: doorState })),
          ),
      );
      return () => {
        subs.forEach((sub) => sub.unsubscribe());
      };
    }, [rxRmf, doors]);

    const handleOnDoorControlClick = React.useCallback(
      (_ev, door: RmfModels.Door, mode: number) =>
        rmfIngress?.doorsApi.postDoorRequestDoorsDoorNameRequestPost(
          {
            mode: mode,
          },
          door.name,
        ),
      [rmfIngress],
    );

    const classes = useStyles();

    return (
      <Window
        ref={ref}
        title="Doors"
        toolbar={
          <IconButton
            aria-label="view mode"
            className={classes.itemIcon}
            onClick={() => setIsCellView(!isCellView)}
          >
            {isCellView ? <ViewListIcon /> : <ViewModuleIcon />}
          </IconButton>
        }
        {...otherProps}
      >
        <Grid container direction="row" spacing={1} className={classes.grid}>
          {isCellView ? (
            doors.map((door) => {
              return (
                <Grid item xs={4} key={door.door.name}>
                  <DoorCell
                    door={door}
                    doorMode={doorStates[door.door.name]?.current_mode.value}
                    onDoorControlClick={handleOnDoorControlClick}
                  />
                </Grid>
              );
            })
          ) : (
            <DoorTable
              doors={doors}
              doorStates={doorStates}
              onDoorControlClick={handleOnDoorControlClick}
            />
          )}
        </Grid>
        {children}
      </Window>
    );
  },
);

const useStyles = makeStyles((theme) => ({
  grid: {
    padding: theme.spacing(2),
  },
  itemIcon: {
    color: theme.palette.getContrastText(theme.palette.primary.main),
  },
}));

export const doorsWindowClass = new WindowClass('Door', DoorsWindow, {
  x: 0,
  y: 0,
  w: 4,
  h: 4,
  minW: 4,
  minH: 4,
});

allWindows['doors'] = doorsWindowClass;
