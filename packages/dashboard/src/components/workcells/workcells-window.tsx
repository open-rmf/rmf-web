import { Grid, IconButton, makeStyles, Typography } from '@material-ui/core';
import ViewListIcon from '@material-ui/icons/ViewList';
import ViewModuleIcon from '@material-ui/icons/ViewModule';
import { Dispenser, Ingestor } from 'api-client';
import React from 'react';
import { useAsync, Window, WorkcellCell, WorkcellTable } from 'react-components';
import * as RmfModels from 'rmf-models';
import { throttleTime } from 'rxjs';
import { v4 as uuidv4 } from 'uuid';
import { RmfIngressContext, RxRmfContext } from '../rmf-app';
import { allWindows, ManagedWindowProps, WindowClass } from '../window';

export const WorkcellsWindow: React.FC<ManagedWindowProps> = React.forwardRef(
  ({ key, children, ...otherProps }, ref) => {
    const safeAsync = useAsync();
    const rmfIngress = React.useContext(RmfIngressContext);
    const rxRmf = React.useContext(RxRmfContext);
    const [dispensers, setDispensers] = React.useState<Dispenser[]>([]);
    const [dispenserStates, setDispenserStates] = React.useState<
      Record<string, RmfModels.DispenserState>
    >({});
    const [ingestors, setIngestors] = React.useState<Ingestor[]>([]);
    const [ingestorStates, setIngestorStates] = React.useState<
      Record<string, RmfModels.IngestorState>
    >({});

    React.useEffect(() => {
      if (!rmfIngress) return;
      (async () => {
        const dispensers = (await safeAsync(rmfIngress.dispensersApi.getDispensersDispensersGet()))
          .data;
        setDispensers(dispensers);
        const ingestors = (await safeAsync(rmfIngress.ingestorsApi.getIngestorsIngestorsGet()))
          .data;
        setIngestors(ingestors);
      })();
    }, [safeAsync, rmfIngress]);

    React.useEffect(() => {
      if (!rxRmf) return;
      const dispenserSubs = dispensers.map((dispenser) =>
        rxRmf
          .dispenserStates(dispenser.guid)
          .pipe(throttleTime(1000))
          .subscribe(
            (dispenserState) =>
              dispenserState &&
              setDispenserStates((prev) => ({ ...prev, [dispenser.guid]: dispenserState })),
          ),
      );
      return () => {
        dispenserSubs.forEach((sub) => sub.unsubscribe());
      };
    }, [rxRmf, dispensers]);

    React.useEffect(() => {
      if (!rxRmf) return;
      const ingestorSubs = ingestors.map((ingestor) =>
        rxRmf
          .ingestorStates(ingestor.guid)
          .pipe(throttleTime(1000))
          .subscribe(
            (ingestorState) =>
              ingestorState &&
              setIngestorStates((prev) => ({ ...prev, [ingestor.guid]: ingestorState })),
          ),
      );
      return () => {
        ingestorSubs.forEach((sub) => sub.unsubscribe());
      };
    }, [rxRmf, ingestors]);

    const classes = useStyles();
    const [isCellView, setIsCellView] = React.useState(true);

    return (
      <Window
        ref={ref}
        key={key}
        title="Workcells"
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
        {isCellView ? (
          <React.Fragment>
            <div className={classes.cellContainer}>
              <Typography variant="h6">Dispensers</Typography>
              <Grid container direction="row" spacing={1}>
                {dispensers.length > 0
                  ? dispensers.map((dispenser, i) => {
                      const state: RmfModels.DispenserState | undefined =
                        dispenserStates[dispenser.guid];
                      return (
                        <Grid item xs={4} key={`${dispenser.guid}_${i}`}>
                          <WorkcellCell
                            guid={dispenser.guid}
                            requestGuidQueue={state?.request_guid_queue}
                            secondsRemaining={state?.seconds_remaining}
                          />
                        </Grid>
                      );
                    })
                  : null}
              </Grid>
            </div>
            <div className={classes.cellContainer}>
              <Typography variant="h6">Ingesters</Typography>
              <Grid container direction="row" spacing={1}>
                {ingestors.length > 0
                  ? ingestors.map((ingestor, i) => {
                      const state: RmfModels.IngestorState | undefined =
                        ingestorStates[ingestor.guid];
                      return (
                        <Grid item xs={4} key={`${ingestor.guid}_${i}`}>
                          <WorkcellCell
                            guid={ingestor.guid}
                            requestGuidQueue={state?.request_guid_queue}
                            secondsRemaining={state?.seconds_remaining}
                          />
                        </Grid>
                      );
                    })
                  : null}
              </Grid>
            </div>
          </React.Fragment>
        ) : (
          <React.Fragment>
            {dispensers.length > 0 ? (
              <div>
                <Typography variant="h6" className={classes.subPanelHeader}>
                  Dispenser Table
                </Typography>
                <WorkcellTable workcells={dispensers} workcellStates={dispenserStates} />
              </div>
            ) : null}
            {ingestors.length > 0 ? (
              <div>
                <Typography variant="h6" className={classes.subPanelHeader}>
                  Ingestor Table
                </Typography>
                <WorkcellTable workcells={ingestors} workcellStates={ingestorStates} />
              </div>
            ) : null}
          </React.Fragment>
        )}
        {children}
      </Window>
    );
  },
);

const useStyles = makeStyles((theme) => ({
  itemIcon: {
    color: theme.palette.getContrastText(theme.palette.primary.main),
  },
  cellContainer: {
    paddingLeft: '1rem',
    paddingRight: '1rem',
    paddingBottom: '1rem',
  },
  subPanelHeader: {
    marginLeft: '1rem',
  },
}));

export const workcellsWindowClass: WindowClass = {
  Component: WorkcellsWindow,
  createLayout: (_bp, layout = {}) => ({
    i: uuidv4(),
    x: 0,
    y: 0,
    w: 4,
    h: 4,
    minW: 4,
    minH: 4,
    ...layout,
  }),
};

allWindows['workcells'] = workcellsWindowClass;
