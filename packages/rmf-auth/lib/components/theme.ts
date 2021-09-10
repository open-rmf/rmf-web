// it could be your App.tsx file or theme file that is included in your tsconfig.json
import { Theme } from '@material-ui/core/styles';

declare module '@material-ui/styles/defaultTheme' {
  // eslint-disable-next-line @typescript-eslint/no-empty-interface (remove this line if you don't have the rule enabled)
  interface DefaultTheme extends Theme {}
}
