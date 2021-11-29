import { styled } from '@mui/material';

export const defaultReportClasses = {
  table: 'default-report-table',
};
export const DefaultReportContainer = styled('div')(() => ({
  [`& .${defaultReportClasses.table}`]: {
    overflowY: 'scroll',
    paddingTop: '20px',
  },
}));

export interface DefaultReportQueryPayload {
  toLogDate?: unknown | null;
  fromLogDate?: unknown | null;
  offset?: number | null;
  limit?: number | null;
}

export interface DefaultLogTableProps {
  tableSize?: number;
  addMoreRows?(): void;
}
