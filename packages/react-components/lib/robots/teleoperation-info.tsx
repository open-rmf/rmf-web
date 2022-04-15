import { Divider, Grid, styled, useTheme, Popover, TextField, Box, Switch } from '@mui/material';
import { height } from '@mui/system';
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
  const [client, setClient] = React.useState<mqtt.MqttClient | null>();
  const [anchorEl, setAnchorEl] = React.useState<HTMLButtonElement | null>(null);
  const [companyId, setCompanyId] = React.useState('04e17fb6-63cb-41f8-811b-8f40fc7ca729');
  const [robotId, setRobotId] = React.useState('56565883-0c02-4fc3-80bb-b547737005da');
  const [accessKey, setAccessKey] = React.useState('528affd9574e4ae59b568958a9d3a6d1');
  const [manualControl, setManualControl] = React.useState(false);
  const [frontStreamUrl, setFrontStreamUrl] = React.useState<string | null>();
  const [leftStreamUrl, setLeftStreamUrl] = React.useState<string | null>();
  const [rightStreamUrl, setRightStreamUrl] = React.useState<string | null>();

  const handleClick = (event: React.MouseEvent<HTMLButtonElement>) => {
    setAnchorEl(event.currentTarget);
  };

  const handleClose = () => {
    setAnchorEl(null);
  };

  const open = Boolean(anchorEl);
  const id = open ? 'simple-popover' : undefined;

  // TODO(BH): Make this globally configurable
  const name = robotName.trim();

  React.useEffect(() => {
    const payload = {
      robotId: robotId,
      accessKey: accessKey,
    };
    const headers = { Accept: 'application/json', 'Content-Type': 'application/json' };
    const requestOptions: RequestInit = {
      method: 'POST',
      headers: headers,
      body: JSON.stringify(payload),
      mode: 'no-cors',
    };

    // console.log('login data are');
    // // Fetch login info
    // fetch('http://dev.robotmanager.io:83/api/v2/mqtt/loginInfo', requestOptions)
    // .then((response) => {
    //   return response.text();
    // })
    // .then((data) => {
    //   console.log(data);
    // })
    // .catch((error) => {
    //   console.log(error);
    // });

    const client = mqtt.connect({
      hostname: 'dev.mqtt.robotmanager.io',
      port: 9001,
      protocol: 'ws',
      username: 'Admin',
      password: 'Admin123',
      keepalive: 60,
      // clientId: robotId+'_API',
      clean: true,
    });

    setClient(client);

    client.once('connect', function () {
      console.log('Connected to mqqt server');
      // Subscribe to video channel
      const vcTopic = '/robot/vc/update/' + companyId;
      client.subscribe(vcTopic, function (err, granted) {
        console.log(granted);
        if (err) {
          console.log(err);
        }
      });
      const stTopic = '/robot/status/' + companyId;
      client.subscribe(stTopic, function (err, granted) {
        console.log(granted);
        if (err) {
          console.log(err);
        }
      });
      const controlStTopic = '/robot/control/status/' + companyId;
      client.subscribe(controlStTopic, function (err, granted) {
        console.log(granted);
        if (err) {
          console.log(err);
        }
      });
    });

    client.on('message', function (topic, message) {
      // message is Buffer
      console.log('recived a msg of topic ', topic);
      parseVC(message.toString());
      console.log(JSON.parse(message.toString()));
      client.end();
    });

    client.on('error', function (err) {
      // message is Buffer
      console.log(err);
    });
  }, []);

  const parseVC = (message: string) => {
    console.log('vc message: ', message);
    const jsonMsg = JSON.parse(message);
    if (jsonMsg.robotId === robotId && jsonMsg.companyId === companyId) {
      for (const stream of jsonMsg.streams) {
        if (stream['name'] === 'Front' && stream['videoAvailable']) {
          setFrontStreamUrl(stream['link']);
        }
        if (stream['name'] === 'Right' && stream['videoAvailable']) {
          setRightStreamUrl(stream['link']);
        }
        if (stream['name'] === 'Left' && stream['videoAvailable']) {
          setLeftStreamUrl(stream['link']);
        }
      }
    }
  };

  const publishControl = async (direction: string) => {
    if (client && client.connected) {
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
          robotId: robotId,
          companyId: companyId,
          timestamp: Date.now(),
          payload: { x: x, y: y },
        }),
      );
    } else {
      console.log('Cant  control robot, NOT CONNECTED');
      return;
    }
  };

  const handleChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setManualControl(event.target.checked);

    // Send mode request to robot
    const topic = '/rm/mode/' + companyId;
    if (client && client.connected) {
      client.publish(
        topic,
        JSON.stringify({
          id: Date.now(),
          robotId: robotId,
          companyId: companyId,
          timestamp: Date.now(),
          teleoperation: !manualControl,
        }),
      );
    }
  };

  return (
    <StyledDiv style={{ height: '100%', width: '100%' }}>
      <Divider />
      <Grid container style={{ height: '100%', width: '100%' }}>
        <Switch checked={manualControl} onChange={handleChange}></Switch>

        <Grid container item xs={12} justifyContent="center">
          {frontStreamUrl ? (
            <>
              <iframe
                allow="fullscreen; display-capture; autoplay"
                src={frontStreamUrl}
                style={{ height: '95%', width: '100%', border: '0px' }}
              ></iframe>
            </>
          ) : null}
          <Divider />
          {manualControl ? (
            <>
              <button onClick={() => publishControl('LEFT')}>Turn Left</button>
              <button onClick={() => publishControl('DRIVE')}>Drive</button>
              <button onClick={() => publishControl('REVERSE')}>Reverse</button>
              <button onClick={() => publishControl('RIGHT')}>Turn Right</button>
            </>
          ) : (
            <>
              <button onClick={() => publishControl('LEFT')} disabled={true}>
                Turn Left
              </button>
              <button onClick={() => publishControl('DRIVE')} disabled={true}>
                Drive
              </button>
              <button onClick={() => publishControl('REVERSE')} disabled={true}>
                Reverse
              </button>
              <button onClick={() => publishControl('RIGHT')} disabled={true}>
                Turn Right
              </button>
            </>
          )}
        </Grid>
        <button onClick={handleClick}>Edit robot info</button>
        <Popover
          id={id}
          open={open}
          anchorEl={anchorEl}
          onClose={handleClose}
          anchorOrigin={{
            vertical: 'bottom',
            horizontal: 'left',
          }}
        >
          <Box
            component="form"
            sx={{
              '& > :not(style)': { m: 1, width: '25ch' },
            }}
            noValidate
            autoComplete="off"
          >
            <TextField
              id="outlined-basic"
              label="Robot ID"
              value={robotId}
              variant="outlined"
              onChange={(ev) => setRobotId(ev.target.value)}
            />
            <TextField
              id="filled-basic"
              label="Access Key"
              value={accessKey}
              variant="outlined"
              onChange={(ev) => setAccessKey(ev.target.value)}
            />
            <TextField
              id="filled-basic"
              label="Company ID"
              value={companyId}
              variant="outlined"
              onChange={(ev) => setCompanyId(ev.target.value)}
            />
          </Box>
        </Popover>
      </Grid>
    </StyledDiv>
  );
}
