import React from 'react';
import { DefaultReportQueryPayload, defaultReportStyles } from '../default-report-interface';
import { DefaultDatesForm } from '../default-dates-form';
import { LiftStateReportTable, LiftStateRowsType } from './lift-state-report-table';

export interface LiftStateReportProps {
  getLogs: (data: DefaultReportQueryPayload) => Promise<LiftStateRowsType>;
}

export const LiftStateReport = (props: LiftStateReportProps): React.ReactElement => {
  const { getLogs } = props;
  const [logs, setLogs] = React.useState<LiftStateRowsType>([]);
  const [lastSearchParams, setLastSearchParams] = React.useState<DefaultReportQueryPayload>({});

  const classes = defaultReportStyles();

  const searchLogs = async (payload: DefaultReportQueryPayload) => {
    setLastSearchParams(payload);
    setLogs(await getLogs(payload));
  };

  const getMoreLogs = async () => {
    setLogs(logs.concat(await getLogs({ ...lastSearchParams, offset: logs.length })));
  };

  return (
    <>
      <DefaultDatesForm search={searchLogs} />
      <div className={classes.table}>
        {logs.length !== 0 && (
          <LiftStateReportTable rows={logs} tableSize={'48vh'} addMoreRows={getMoreLogs} />
        )}
      </div>
    </>
  );
};

export default LiftStateReport;
