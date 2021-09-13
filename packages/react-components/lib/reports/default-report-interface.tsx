import { makeStyles } from '@material-ui/styles';

export const defaultReportStyles = makeStyles(() => ({
  table: {
    overflowY: 'scroll',
    paddingTop: '20px',
  },
}));

export interface DefaultReportQueryPayload {
  toLogDate?: Date | null;
  fromLogDate?: Date | null;
  offset?: number | null;
  limit?: number | null;
}

export interface DefaultLogTableProps {
  tableSize?: number;
  addMoreRows?(): void;
}
