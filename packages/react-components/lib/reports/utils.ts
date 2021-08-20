export interface ReportConfigProps {
  fromLogDate?: Date;
  toLogDate?: Date;
  onSelectFromDate?: (date: any) => void;
  onSelectToDate?: (date: any) => void;
}
