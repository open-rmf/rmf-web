import React from 'react';
import { DefaultReportQueryPayload, defaultReportStyles } from '../default-report-interface';
import { DefaultDatesForm } from '../default-dates-form';
import { DoorStateReportTable, DoorStateRowsType } from './door-state-report-table';

export interface DoorStateReportProps {
  getLogs: (data: DefaultReportQueryPayload) => Promise<DoorStateRowsType>;
}

export const DoorStateReport = (props: DoorStateReportProps): React.ReactElement => {
  const { getLogs } = props;
  const [logs, setLogs] = React.useState<DoorStateRowsType>([]);
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
          <DoorStateReportTable rows={logs} tableSize={'48vh'} addMoreRows={getMoreLogs} />
        )}
      </div>
    </>
  );
};

export default DoorStateReport;
