import { MaterialUiPickersDate } from '@material-ui/pickers/typings/date';

export interface ReportConfigProps {
  fromLogDate?: MaterialUiPickersDate;
  toLogDate?: MaterialUiPickersDate;
  onSelectFromDate?: (date: MaterialUiPickersDate) => void;
  onSelectToDate?: (date: MaterialUiPickersDate) => void;
}
