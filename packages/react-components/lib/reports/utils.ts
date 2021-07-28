import { MaterialUiPickersDate } from '@material-ui/pickers/typings/date';

export interface ConfigProps {
  fromLogDate?: MaterialUiPickersDate;
  toLogDate?: MaterialUiPickersDate;
  onSelectFromDate?: (date: MaterialUiPickersDate) => void;
  onSelectToDate?: (date: MaterialUiPickersDate) => void;
}
