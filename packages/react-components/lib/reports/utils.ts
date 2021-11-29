export interface ReportConfigProps {
  fromLogDate?: Date;
  toLogDate?: Date;
  onSelectFromDate?: (date: unknown) => void;
  onSelectToDate?: (date: unknown) => void;
}
