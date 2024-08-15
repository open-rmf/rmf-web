import { Tab, Typography } from '@mui/material';
import React from 'react';
import { Route, Routes, useNavigate } from 'react-router';

import { Authenticator, AuthenticatorContext } from '../services/authenticator';
import { RmfApi } from '../services/rmf-api';
import AppBar, { APP_BAR_HEIGHT } from './appbar';
import Map from './map';
import { UserProfileProvider } from './user-profile-provider';
import { StaticWorkspace } from './workspace';

export * from '../services/rmf-api';

export const RmfApiContext = React.createContext<RmfApi | null>(null);

export interface DashboardHome {}

export interface DashboardTab {
  name: string;
  route: string;
  element: React.ReactNode;
}

export interface RmfDashboardProps {
  apiServerUrl: string;
  trajectoryServerUrl: string;
  authenticator: Authenticator;
  helpLink: string;
  reportIssueLink: string;
  pickupZones: string[];
  cartIds: string[];
  tabs: DashboardTab[];
}

export function RmfDashboard({
  apiServerUrl,
  trajectoryServerUrl,
  authenticator,
  helpLink,
  reportIssueLink,
  pickupZones,
  cartIds,
  tabs,
}: RmfDashboardProps) {
  const rmfApi = React.useMemo(
    () => new RmfApi(apiServerUrl, trajectoryServerUrl, authenticator),
    [apiServerUrl, trajectoryServerUrl, authenticator],
  );
  const [authReady, setAuthReady] = React.useState(false);
  const [tabValue, setTabValue] = React.useState(tabs[0].name);
  const navigate = useNavigate();

  React.useEffect(() => {
    (async () => {
      await authenticator.init();
      setAuthReady(true);
    })();
  }, [authenticator]);

  return (
    authReady && (
      <AuthenticatorContext.Provider value={authenticator}>
        <RmfApiContext.Provider value={rmfApi}>
          <UserProfileProvider>
            <AppBar
              tabs={tabs.map((t) => (
                <Tab
                  key={t.name}
                  sx={{ height: APP_BAR_HEIGHT }}
                  label={<Typography variant="h6">{t.name}</Typography>}
                  value={t.name}
                  onClick={() => navigate(t.route)}
                />
              ))}
              tabValue={tabValue}
              helpLink={helpLink}
              reportIssueLink={reportIssueLink}
              pickupZones={pickupZones}
              cartIds={cartIds}
            ></AppBar>
            <Routes>
              {tabs.map((t) => (
                <Route
                  key={t.name}
                  path={t.route}
                  element={t.element}
                  loader={() => {
                    setTabValue(t.name);
                    return null;
                  }}
                />
              ))}
            </Routes>
          </UserProfileProvider>
        </RmfApiContext.Provider>
      </AuthenticatorContext.Provider>
    )
  );
}
