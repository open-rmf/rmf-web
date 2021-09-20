import React from 'react';
import {
  DefaultReportQueryPayload,
  defaultReportClasses,
  DefaultReportRoot,
} from '../default-report-interface';
import { DefaultDatesForm } from '../default-dates-form';
import { HealthReportTable, HealthRowsType } from './health-report-table';
import { ReportConfigProps } from '../utils';

export interface HealthReportProps extends ReportConfigProps {
  getLogs: (data: DefaultReportQueryPayload) => Promise<HealthRowsType>;
}

export const HealthReport = (props: HealthReportProps): React.ReactElement => {
  const { getLogs, ...otherProps } = props;
  const [logs, setLogs] = React.useState<HealthRowsType>([]);
  const [lastSearchParams, setLastSearchParams] = React.useState<DefaultReportQueryPayload>({});

  const searchLogs = async (payload: DefaultReportQueryPayload) => {
    setLastSearchParams(payload);
    setLogs(await getLogs(payload));
  };

  const getMoreLogs = async () => {
    setLogs(logs.concat(await getLogs({ ...lastSearchParams, offset: logs.length })));
  };

  return (
    <DefaultReportRoot>
      <DefaultDatesForm search={searchLogs} {...otherProps} />
      <div className={defaultReportClasses.table}>
        {logs.length !== 0 && (
          <HealthReportTable rows={logs} tableSize={500} addMoreRows={getMoreLogs} />
        )}
      </div>
    </DefaultReportRoot>
  );
};

export default HealthReport;
