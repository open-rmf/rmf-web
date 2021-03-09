import React from 'react';
import { ReporterState } from './reporter-reducer';
import { LogManagement, DoorStateReport, DoorRowsType } from 'react-components';

interface ReporterBodyProps {
  reporterState: ReporterState;
}

/**
 * Trash
 */
const getLogLabels = () => [
  { label: 'Web Server', value: 'web-server' },
  { label: 'RMF core', value: 'rmf-core' },
];

function randomDate(start: Date, end: Date) {
  return new Date(start.getTime() + Math.random() * (end.getTime() - start.getTime()));
}

const getLogs = () => {
  const rows = [];
  for (let i = 0; i < 500; i++) {
    rows.push({
      message: 'Test' + i,
      level: 'Debug',
      timestamp: randomDate(new Date(2012, 0, 1), new Date()).toISOString(),
    });
  }
  return rows;
};

const getLogsPromise = async () => getLogs();
const getLabelsPromise = async () => getLogLabels();
/**
 * Trash
 */

export const ReporterBody = (props: ReporterBodyProps) => {
  const { reporterState } = props;
  return (
    <>
      {reporterState.queryAllLogs && (
        <LogManagement getLogs={getLogsPromise} getLabels={getLabelsPromise} />
      )}
      {reporterState.showDoorStateReport && (
        <DoorStateReport
          rows={
            [
              {
                name: 'doorA',
                status: 'closed',
                message: 'test',
                timestamp: new Date().toISOString(),
              },
            ] as DoorRowsType
          }
          tableSize={'49vh'}
        />
      )}
      {!reporterState.queryAllLogs && !reporterState.showDoorStateReport && (
        <h1>you should pick a report</h1>
      )}
    </>
  );
};
