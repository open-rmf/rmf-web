import React from 'react';
import {
  DefaultReportQueryPayload,
  StyledDefaultReport,
  defaultReportClasses,
} from '../default-report-interface';
import { DefaultDatesForm } from '../default-dates-form';
import { FleetStateReportTable, FleetStateRowsType } from './fleet-state-report-table';
import { ReportConfigProps } from '../utils';

export interface FleetStateReportProps extends ReportConfigProps {
  getLogs: (data: DefaultReportQueryPayload) => Promise<FleetStateRowsType>;
}

export const FleetStateReport = (props: FleetStateReportProps): React.ReactElement => {
  const { getLogs, ...otherProps } = props;
  const [logs, setLogs] = React.useState<FleetStateRowsType>([]);
  const [lastSearchParams, setLastSearchParams] = React.useState<DefaultReportQueryPayload>({});

  const searchLogs = async (payload: DefaultReportQueryPayload) => {
    setLastSearchParams(payload);
    setLogs(await getLogs(payload));
  };

  const getMoreLogs = async () => {
    setLogs(logs.concat(await getLogs({ ...lastSearchParams, offset: logs.length })));
  };

  return (
    <StyledDefaultReport>
      <DefaultDatesForm search={searchLogs} {...otherProps} />
      <div className={defaultReportClasses.table}>
        {logs.length !== 0 && (
          <FleetStateReportTable rows={logs} tableSize={500} addMoreRows={getMoreLogs} />
        )}
      </div>
    </StyledDefaultReport>
  );
};

export default FleetStateReport;
