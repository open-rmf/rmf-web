import { Divider, Grid, styled, useTheme } from '@mui/material';
import { TeleoperationApi } from 'api-client';
// import { robot_manager_login_url, robot_manager_robot_id, robot_manager_access_key, robot_manager_server_endpoint, robot_manager_server_port, robot_manager_company_id } from '../../../dashboard/src/util/url';
import * as mqtt from 'mqtt';
import React from 'react';

const classes = {
  button: 'robot-info-button',
};
const StyledDiv = styled('div')(() => ({
  [`& .${classes.button}`]: {
    '&:hover': {
      background: 'none',
      cursor: 'default',
    },
  },
}));

export interface TeleoperationInfoProps {
  robotName: string;
  teleoperationApi: TeleoperationApi;
}

export function TeleoperationInfo({
  robotName,
  teleoperationApi,
}: TeleoperationInfoProps): JSX.Element {
  const theme = useTheme();
  const [client, setClient] = React.useState<mqtt.MqttClient | undefined>();
  const [companyId, setCompanyId] = React.useState('04e17fb6-63cb-41f8-811b-8f40fc7ca729');

  // TODO(BH): Make this globally configurable
  const name = robotName.trim();
  const url = 'https://meet.jit.si/' + name;

  React.useEffect(() => {
    const payload = {
      robotId: '56565883-0c02-4fc3-80bb-b547737005da',
      accessKey: '528affd9574e4ae59b568958a9d3a6d1',
    };
    const headers = { Accept: 'application/json', 'Content-Type': 'application/json' };
    const requestOptions: RequestInit = {
      method: 'POST',
      headers: headers,
      body: JSON.stringify(payload),
      mode: 'no-cors',
    };

    fetch('http://dev.robotmanager.io:83/api/v2/mqtt/get-by-robot', requestOptions)
      .then((response) => {
        return response.text();
      })
      .then((data) => {
        console.log(data);
      })
      .catch((error) => {
        console.log(error);
      });
    // const result =  json.loads(response.text);
    // if result is not None and \
    //   len(result['error']) == 0 and \
    //     result['message'] == 'Success':
    // username = result['result']['userName']
    // password = result['result']['password']

    // // Connect to mqtt server
    const client = mqtt.connect('dev.mqtt.robotmanager.io', {
      port: 1883,
      username: 'Admin',
      password: 'Admin123',
    });
    setClient(client);
  }, []);

  const publishControl = async (direction: string) => {
    if (!client) {
      console.log('Cant  control robot, NOT CONNECTED');
      return;
    }

    let x: number = 0;
    let y: number = 0;
    switch (direction) {
      case 'RIGHT':
        x = 1;
        break;
      case 'LEFT':
        x = -1;
        break;
      case 'REVERSE':
        y = -1;
        break;
      default:
        y = 1;
        break;
    }
    client.publish(
      '/rm/move/' + companyId,
      JSON.stringify({
        id: 1,
        robotId: '56565883-0c02-4fc3-80bb-b547737005da',
        companyId: companyId,
        timestamp: Date.now(),
        payload: { x: x, y: y },
      }),
    );
  };

  return (
    <StyledDiv style={{ height: '100%', width: '100%' }}>
      <Divider />
      <Grid container style={{ height: '100%', width: '100%' }}>
        <Grid container item xs={12} justifyContent="center">
          <iframe
            allow="camera; microphone; fullscreen; display-capture; autoplay"
            src={url}
            style={{ height: '95%', width: '100%', border: '0px' }}
          ></iframe>
          <button
            onClick={() =>
              teleoperationApi.joinVideoRoomTeleoperationNameVideoJoinPost(name, {
                target_name: name,
                url: url,
              })
            }
          >
            Join
          </button>
          <button onClick={() => console.log('LEAVE')}>Leave</button>
          <Divider />
          <button onClick={() => publishControl('LEFT')}>Turn Left</button>
          <button onClick={() => publishControl('DRIVE')}>Drive</button>
          <button onClick={() => publishControl('REVERSE')}>Reverse</button>
          <button onClick={() => publishControl('RIGHT')}>Turn Right</button>
        </Grid>
      </Grid>
    </StyledDiv>
  );
}
