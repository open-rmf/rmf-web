import React from 'react';
import { DefaultReportQueryPayload, defaultReportStyles } from '../default-report-interface';
import { DefaultDatesForm } from '../default-dates-form';
import { IngestorStateReportTable, IngestorStateRowsType } from './ingestor-state-report-table';

export interface IngestorStateReportProps {
  getLogs: (data: DefaultReportQueryPayload) => Promise<IngestorStateRowsType>;
}

export const IngestorStateReport = (props: IngestorStateReportProps): React.ReactElement => {
  const { getLogs } = props;
  const [logs, setLogs] = React.useState<IngestorStateRowsType>([]);
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
          <IngestorStateReportTable rows={logs} tableSize={'48vh'} addMoreRows={getMoreLogs} />
        )}
      </div>
    </>
  );
};

export default IngestorStateReport;
