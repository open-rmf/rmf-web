import { LocalizationProvider as MuiLocalizationProvider } from '@mui/lab';
import AdapterDateFns from '@mui/lab/AdapterDateFns';
import React from 'react';

export const TestLocalizationProvider: React.FC = ({ children }) => {
  return <MuiLocalizationProvider dateAdapter={AdapterDateFns}>{children}</MuiLocalizationProvider>;
};
