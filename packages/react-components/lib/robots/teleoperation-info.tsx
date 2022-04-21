import {
  Divider,
  Grid,
  styled,
  useTheme,
  Popover,
  TextField,
  Box,
  Switch,
  FormGroup,
  FormControlLabel,
  IconButton,
} from '@mui/material';
import { height } from '@mui/system';
import * as mqtt from 'mqtt';
import React, { useEffect } from 'react';
import {
  KeyboardArrowLeftSharp,
  KeyboardArrowRightSharp,
  KeyboardArrowUpSharp,
  KeyboardArrowDownSharp,
} from '@mui/icons-material';

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
  name: string;
  id: string;
  companyId: string;
  accessKey: string;
}

export function TeleoperationInfo({
  name,
  id,
  companyId,
  accessKey,
}: TeleoperationInfoProps): JSX.Element {
  const theme = useTheme();
  const [client, setClient] = React.useState<mqtt.MqttClient | null>();
  const [anchorEl, setAnchorEl] = React.useState<HTMLButtonElement | null>(null);
  const [manualControl, setManualControl] = React.useState(false);
  const [frontStreamUrl, setFrontStreamUrl] = React.useState<string | null>();
  const [token, setToken] = React.useState<string | null>();

  const handleClick = (event: React.MouseEvent<HTMLButtonElement>) => {
    setAnchorEl(event.currentTarget);
  };

  const handleClose = () => {
    setAnchorEl(null);
  };

  // TODO(BH): Make this globally configurable
  const open = Boolean(anchorEl);
  const controlTopic = '/rm/mode/' + companyId;
  const vcTopic = '/robot/vc/update/' + companyId;

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
      const msg = JSON.stringify({
        id: 1,
        robotId: id,
        companyId: companyId,
        timestamp: Date.now(),
        payload: { x: x, y: y },
      });
      client.publish('/rm/move/' + companyId, msg);
      setTimeout(function () {
        client.publish(
          '/rm/move/' + companyId,
          JSON.stringify({
            id: 2,
            robotId: id,
            companyId: companyId,
            timestamp: Date.now(),
            payload: { x: 0, y: 0 },
          }),
        );
      }, 1000);
    } else {
      console.log('Cant  control robot, NOT CONNECTED');
      return;
    }
  };

  async function switchControl() {
    if (client && client.connected) {
      const msg = JSON.stringify({
        id: Date.now().toString(),
        robotId: id,
        companyId: companyId,
        timestamp: Date.now().toString(),
        teleoperation: !manualControl,
      });
      console.log(msg);
      client.publish(controlTopic, msg);
    } else {
      console.log('no longer connected');
    }
  }

  useEffect(() => {
    switchControl();
  }, [id, companyId]);

  const getToken = async () => {
    try {
      const response = await fetch('http://dev.robotmanager.io:83/api/v2/login', {
        method: 'POST',
        headers: { Accept: 'application/json', 'Content-Type': 'application/json' },
        body: JSON.stringify({
          username: 'mohamad@openrobotics.org',
          password: 'Abcd123$',
          companyId: companyId,
        }),
        // mode:'cors'
      });
      const data = await response.json();
      setToken(data.token);
    } catch (e) {
      console.log(e);
    }
  };

  useEffect(() => {
    getToken();
  }, [companyId]);

  const getStreamURLs = async () => {
    if (token) {
      try {
        const response = await fetch('http://dev.robotmanager.io:83/api/v2/camera/list/' + id, {
          method: 'GET',
          headers: { Accept: 'application/json', Authorization: 'Bearer ' + token },
        });
        const data = await response.json();
        setFrontStreamUrl(data.result[0].url);
      } catch (e) {
        console.log(e);
      }
    }
  };

  useEffect(() => {
    if (token !== null) {
      getStreamURLs();
    }
  }, [token, id]);

  useEffect(() => {
    // // Fetch mqtt login info
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

    // Connect to mqtt server
    const client = mqtt.connect({
      hostname: 'dev.mqtt.robotmanager.io',
      port: 9001,
      protocol: 'ws',
      username: 'Admin',
      password: 'Admin123',
      keepalive: 60,
      clientId: 'mqttjs_' + Math.random().toString(16),
      clean: true,
    });

    setClient(client);
    client.on('disconnect', function (packet) {
      console.log('disconnected');
      console.log(packet);
    });

    client.on('close', function () {
      console.log('connection closed');
    });

    client.on('error', function (packet) {
      console.log(packet);
    });

    client.on('connect', function () {
      console.log('Connected to mqqt server');

      // Subscribe to video channel
      client.subscribe(vcTopic, function (err) {
        if (err) {
          console.log(err);
        }
      });

      client.publish(
        '/rm/vc/request/' + companyId,
        JSON.stringify({
          id: Date.now().toString(),
          timestamp: Date.now().toString(),
          robotId: id,
          companyId: companyId,
          enableVideo: true,
          enableAudio: true,
          hostUrl: 'http://video.robotmanager.io',
          token: 'hjkkjfehjk', // TODO (MH) ??
        }),
      );

      // // Subscribe to robot status
      // const stTopic = '/robot/status/' + companyId;
      // client.subscribe(stTopic, function (err) {
      //   if (err) {
      //     console.log(err);
      //   }
      // });

      // Subscribe to control startus
      const controlStTopic = '/robot/control/status/' + companyId;
      client.subscribe(controlStTopic, function (err) {
        if (err) {
          console.log(err);
        }
      });

      client.subscribe(controlTopic, function (err) {
        if (err) {
          console.log(err);
        }
      });

      const mvTopic = '/rm/move/' + companyId;
      client.subscribe(mvTopic, function (err) {
        if (err) {
          console.log(err);
        }
      });

      client.subscribe(vcTopic, function (err) {
        if (err) {
          console.log(err);
        }
      });
    });

    client.on('message', function (topic, message) {
      //if (topic === vcTopic) {
      //  parseVC(message.toString());
      //}
      // console.log(JSON.parse(message.toString()));
    });

    client.on('error', function (err) {
      // message is Buffer
      console.log(err);
    });
  }, []);

  const handleChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setManualControl(event.target.checked);
    switchControl();
  };

  return (
    <StyledDiv style={{ height: '100%', width: '100%' }}>
      <Divider />
      <Grid container style={{ height: '100%', width: '100%' }}>
        <FormGroup>
          <FormControlLabel
            control={<Switch checked={manualControl} onChange={handleChange}></Switch>}
            label="Manual control"
          />
        </FormGroup>

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
        </Grid>
        <Grid container style={{ height: '185px' }}>
          <Grid container item xs={12} justifyContent="center">
            <IconButton
              onClick={() => publishControl('DRIVE')}
              disabled={!manualControl}
              size="large"
            >
              <KeyboardArrowUpSharp />
            </IconButton>
          </Grid>
          <Grid container item xs={12} justifyContent="center">
            <IconButton
              onClick={() => publishControl('LEFT')}
              disabled={!manualControl}
              size="large"
            >
              <KeyboardArrowLeftSharp />
            </IconButton>
            <IconButton
              onClick={() => publishControl('RIGHT')}
              disabled={!manualControl}
              size="large"
            >
              <KeyboardArrowRightSharp />
            </IconButton>
          </Grid>
          <Grid container item xs={12} justifyContent="center">
            <IconButton
              onClick={() => publishControl('REVERSE')}
              disabled={!manualControl}
              size="large"
            >
              <KeyboardArrowDownSharp />
            </IconButton>
          </Grid>
        </Grid>
      </Grid>
    </StyledDiv>
  );
}
