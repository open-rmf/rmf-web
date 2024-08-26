import { ThemeProvider } from '@mui/material';

export type MuiTheme = React.ComponentProps<typeof ThemeProvider>['theme'];

export interface DashboardThemes {
  default: MuiTheme;
  dark?: MuiTheme;
}
