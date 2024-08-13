import React from 'react';

import { RmfApi } from '../services/rmf-api';

const useGetUsername = (rmfApi: RmfApi | undefined) => {
  const [username, setUsername] = React.useState<string | null>(null);

  React.useEffect(() => {
    if (!rmfApi) {
      return;
    }

    const fetchUsername = async () => {
      try {
        const user = (await rmfApi.defaultApi.getUserUserGet()).data;
        setUsername(user.username);
      } catch (e) {
        console.log(`error getting username: ${(e as Error).message}`);
      }
    };

    fetchUsername();
  }, [rmfApi]);

  return username;
};

export default useGetUsername;
