import { LocalizationProvider as MuiLocalizationProvider } from '@mui/x-date-pickers/LocalizationProvider';
import { AdapterDateFns } from '@mui/x-date-pickers/AdapterDateFns';
import { Locale } from 'date-fns';
import React from 'react';

export const LocaleContext = React.createContext<string | null>(null);

// A wrapper over @mui/x-date-pickers/LocalizationProvider that detects the browser's and
// dynamically loads the appropriate locale.
export const LocalizationProvider: React.FC<React.PropsWithChildren> = ({ children }) => {
  const [locale, setLocale] = React.useState<Locale | null>(null);

  React.useEffect(() => {
    (async () => {
      for (const lang of navigator.languages) {
        try {
          setLocale((await import(`date-fns/locale/${lang}/index.js`)).default);
          return;
        } catch {
          continue;
        }
      }
      setLocale((await import(`date-fns/locale/en-US`)).default);
    })();
  }, []);

  return (
    locale && (
      <MuiLocalizationProvider dateAdapter={AdapterDateFns} adapterLocale={locale}>
        {children}
      </MuiLocalizationProvider>
    )
  );
};

export default LocalizationProvider;
