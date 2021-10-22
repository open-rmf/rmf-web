import { Grid, IconButton, makeStyles } from '@material-ui/core';
import ViewListIcon from '@material-ui/icons/ViewList';
import ViewModuleIcon from '@material-ui/icons/ViewModule';
import React from 'react';
import {
  LiftCell as LiftCell_,
  LiftCellProps,
  LiftTable,
  useAsync,
  Window,
} from 'react-components';
import * as RmfModels from 'rmf-models';
import { throttleTime } from 'rxjs';
import { RmfIngressContext, RxRmfContext } from '../rmf-app';
import { allWindows, ManagedWindowProps, WindowClass } from '../window';

const LiftCell = React.memo(LiftCell_);

export const LiftsWindow: React.FC<ManagedWindowProps> = React.forwardRef(
  ({ children, ...otherProps }, ref) => {
    const safeAsync = useAsync();
    const rmfIngress = React.useContext(RmfIngressContext);
    const rxRmf = React.useContext(RxRmfContext);
    const [lifts, setLifts] = React.useState<RmfModels.Lift[]>([]);
    const [liftStates, setLiftStates] = React.useState<Record<string, RmfModels.LiftState>>({});

    React.useEffect(() => {
      if (!rmfIngress) return;
      (async () => {
        const buildingMap = (await safeAsync(rmfIngress.buildingApi.getBuildingMapBuildingMapGet()))
          .data as RmfModels.BuildingMap;
        setLifts(buildingMap.lifts);
      })();
    }, [safeAsync, rmfIngress]);

    React.useEffect(() => {
      if (!rxRmf) return;
      const subs = lifts.map((lift) =>
        rxRmf
          .liftStates(lift.name)
          .pipe(throttleTime(1000))
          .subscribe(
            (liftState) =>
              liftState && setLiftStates((prev) => ({ ...prev, [lift.name]: liftState })),
          ),
      );
      return () => {
        subs.forEach((sub) => sub.unsubscribe());
      };
    }, [rxRmf, lifts]);

    const handleLiftRequestSubmit = React.useCallback<Required<LiftCellProps>['onRequestSubmit']>(
      (_ev, lift, doorState, requestType, destination) =>
        rmfIngress?.liftsApi.postLiftRequestLiftsLiftNameRequestPost(
          {
            destination,
            request_type: requestType,
            door_mode: doorState,
          },
          lift.name,
        ),
      [rmfIngress],
    );

    const classes = useStyles();
    const [isCellView, setIsCellView] = React.useState(true);

    return (
      <Window
        ref={ref}
        title="Lifts"
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
        <Grid className={classes.grid} container direction="row" spacing={1}>
          {isCellView ? (
            lifts.map((lift, i) => {
              const state: RmfModels.LiftState | undefined = liftStates[lift.name];
              return (
                <Grid item xs={4} key={`${lift.name}_${i}`}>
                  <LiftCell
                    lift={lift}
                    doorState={state?.door_state}
                    motionState={state?.motion_state}
                    destinationFloor={state?.destination_floor}
                    currentFloor={state?.current_floor}
                    onRequestSubmit={handleLiftRequestSubmit}
                  />
                </Grid>
              );
            })
          ) : (
            <LiftTable
              lifts={lifts}
              liftStates={liftStates}
              onRequestSubmit={handleLiftRequestSubmit}
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

export const liftsWindowClass = new WindowClass('Lifts', LiftsWindow, {
  x: 0,
  y: 0,
  w: 4,
  h: 4,
  minW: 4,
  minH: 4,
});

allWindows['lifts'] = liftsWindowClass;
