import { Grid, Slider, Typography } from '@mui/material';
import { makeStyles } from '@mui/styles';
import GestureIcon from '@mui/icons-material/Gesture';
import Leaflet from 'leaflet';
import React from 'react';
import ReactDOM from 'react-dom';
import { MapControl, MapControlProps, withLeaflet } from 'react-leaflet';
import { PositiveIntField } from '../form-inputs';

const useStyles = makeStyles((theme) => ({
  root: {
    width: '100%',
    height: '100%',
  },
  container: {
    padding: theme.spacing(2),
  },
  slider: {
    width: 200,
    verticalAlign: 'middle',
  },
  textField: {
    width: '3em',
  },
}));

export interface TrajectoryTimeControlProps extends MapControlProps {
  /**
   * How far ahead to show the trajectory, in ms.
   */
  value: number;
  /**
   * The min value on the slider, note that it is possible for the value to be less than this by
   * inputting directly into the text field.
   */
  min: number;
  /**
   * The max value on the slider, note that it is possible for the value to be more than this by
   * inputting directly into the text field.
   */
  max: number;
  onChange?: (ev: React.ChangeEvent, value: number) => void;
}

function Component({ value, min, max, onChange }: TrajectoryTimeControlProps) {
  const classes = useStyles();
  const [open, setOpen] = React.useState(false);

  const getValueFormat = (value: number) => `${value / 60000}`;

  return (
    <div
      className={classes.root}
      onMouseEnter={() => setOpen(true)}
      onMouseLeave={() => setOpen(false)}
      aria-label="trajectory time control"
      aria-haspopup={true}
    >
      <div className={classes.container} style={{ display: open ? 'block' : 'none' }}>
        <Typography id="trajectory-time">Trajectory Time (min)</Typography>
        <Grid container spacing={2} alignItems="center">
          <Grid item>
            <Slider
              value={value}
              min={min}
              step={60000}
              max={max}
              getAriaValueText={getValueFormat}
              valueLabelFormat={getValueFormat}
              valueLabelDisplay="auto"
              aria-labelledby="trajectory-time"
              className={classes.slider}
              onChange={onChange as React.ComponentPropsWithoutRef<typeof Slider>['onChange']}
            />
          </Grid>
          <Grid item>
            <PositiveIntField
              value={value / 60000}
              margin="dense"
              className={classes.textField}
              onChange={(ev, newValue) => onChange && onChange(ev, newValue * 60000)}
            />
          </Grid>
        </Grid>
      </div>
      <GestureIcon
        style={{ verticalAlign: 'middle', display: open ? 'none' : 'block' }}
        fontSize="large"
      />
    </div>
  );
}

export class BaseTrajectoryTimeControl extends MapControl<TrajectoryTimeControlProps> {
  constructor(props: TrajectoryTimeControlProps) {
    super(props);
    this._container = Leaflet.DomUtil.create('div');
    this._container.className = 'leaflet-control-layers';
  }

  createLeafletElement(props: TrajectoryTimeControlProps): Leaflet.Control {
    const LeafletControl = Leaflet.Control.extend({
      onAdd: () => {
        ReactDOM.render(<Component {...props} />, this._container);
        // FIXME: react <= 16 installs event handlers on the root document. Stopping propagation
        // of the events on the container will stop react from receiving these events, but not
        // stopping them causes them to propagate to leaflet, causing weird behavior when
        // interacting with the widget.
        // Leaflet.DomEvent.disableClickPropagation(this._container);
        return this._container;
      },
    });
    return new LeafletControl(props);
  }

  updateLeafletElement(
    _fromProps: TrajectoryTimeControlProps,
    toProps: TrajectoryTimeControlProps,
  ): void {
    ReactDOM.render(<Component {...toProps} />, this._container);
  }

  private _container: HTMLElement;
}

export const TrajectoryTimeControl = withLeaflet(BaseTrajectoryTimeControl);

export default TrajectoryTimeControl;
