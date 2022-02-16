import { LocalizationProvider as MuiLocalizationProvider } from '@mui/lab';
import AdapterDateFns from '@mui/lab/AdapterDateFns';
import { Locale } from 'date-fns';
import React from 'react';

export const LocaleContext = React.createContext<string | null>(null);

export const LocalizationProvider: React.FC = ({ children }) => {
  const [locale, setLocale] = React.useState<Locale | null>(null);

  React.useEffect(() => {
    (async () => {
      for (const lang of navigator.languages) {
        try {
          setLocale((await import(`date-fns/locale/${lang}`)).default);
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
      <MuiLocalizationProvider dateAdapter={AdapterDateFns} locale={locale}>
        {children}
      </MuiLocalizationProvider>
    )
  );
};

export default LocalizationProvider;
