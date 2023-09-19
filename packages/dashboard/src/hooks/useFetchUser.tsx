import React from 'react';
import { RmfIngress } from '../components/rmf-app';

const useGetUsername = (rmf: RmfIngress | undefined) => {
  const [username, setUsername] = React.useState<string | null>(null);

  React.useEffect(() => {
    if (!rmf) {
      return;
    }

    const fetchUsername = async () => {
      try {
        const user = (await rmf.defaultApi.getUserUserGet()).data;
        setUsername(user.username);
      } catch (e) {
        console.log(`error getting username: ${(e as Error).message}`);
      }
    };

    fetchUsername();
  }, [rmf]);

  return username;
};

export default useGetUsername;
