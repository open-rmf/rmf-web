import React from 'react';
import {
  DefaultReportQueryPayload,
  defaultReportClasses,
  DefaultReportContainer,
} from '../default-report-interface';
import { DefaultDatesForm } from '../default-dates-form';
import { DispenserStateReportTable, DispenserStateRowsType } from './dispenser-state-report-table';
import { ReportConfigProps } from '../utils';

export interface DispenserStateReportProps extends ReportConfigProps {
  getLogs: (data: DefaultReportQueryPayload) => Promise<DispenserStateRowsType>;
}

export const DispenserStateReport = (props: DispenserStateReportProps): React.ReactElement => {
  const { getLogs, ...otherProps } = props;
  const [logs, setLogs] = React.useState<DispenserStateRowsType>([]);
  const [lastSearchParams, setLastSearchParams] = React.useState<DefaultReportQueryPayload>({});

  const searchLogs = async (payload: DefaultReportQueryPayload) => {
    setLastSearchParams(payload);
    setLogs(await getLogs(payload));
  };

  const getMoreLogs = async () => {
    setLogs(logs.concat(await getLogs({ ...lastSearchParams, offset: logs.length })));
  };

  return (
    <DefaultReportContainer>
      <DefaultDatesForm search={searchLogs} {...otherProps} />
      <div className={defaultReportClasses.table}>
        {logs.length !== 0 && (
          <DispenserStateReportTable rows={logs} tableSize={500} addMoreRows={getMoreLogs} />
        )}
      </div>
    </DefaultReportContainer>
  );
};

export default DispenserStateReport;
