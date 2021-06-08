import React from 'react';
import { DefaultReportQueryPayload, defaultReportStyles } from '../default-report-interface';
import { DefaultDatesForm } from '../default-dates-form';
import { TaskStateReportTable, TaskStateRowsType } from './task-state-report-table';

export interface TaskStateReportProps {
  getLogs: (data: DefaultReportQueryPayload) => Promise<TaskStateRowsType>;
}

export const TaskStateReport = (props: TaskStateReportProps): React.ReactElement => {
  const { getLogs } = props;
  const [logs, setLogs] = React.useState<TaskStateRowsType>([]);
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
          <TaskStateReportTable rows={logs} tableSize={'48vh'} addMoreRows={getMoreLogs} />
        )}
      </div>
    </>
  );
};

export default TaskStateReport;
