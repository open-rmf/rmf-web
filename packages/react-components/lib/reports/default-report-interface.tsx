import { makeStyles } from '@material-ui/core';

export const defaultReportStyles = makeStyles(() => ({
  table: {
    overflowY: 'scroll',
    paddingTop: '20px',
  },
}));

export interface DefaultReportQueryPayload {
  toLogDate?: moment.Moment | null;
  fromLogDate?: moment.Moment | null;
  offset?: number | null;
  limit?: number | null;
}

export interface DefaultLogTableProps {
  tableSize?: string; // units vh or rem
  addMoreRows(): void;
}
