import { styled } from '@mui/material';

export const defaultReportClasses = {
  table: 'default-report-table',
};
export const DefaultReportRoot = styled('div')(() => ({
  [`& .${defaultReportClasses.table}`]: {
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
