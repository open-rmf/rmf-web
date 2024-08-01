import FullscreenIcon from '@mui/icons-material/Fullscreen';
import LayersIcon from '@mui/icons-material/Layers';
import ZoomInIcon from '@mui/icons-material/ZoomIn';
import ZoomOutIcon from '@mui/icons-material/ZoomOut';
import {
  Box,
  Checkbox,
  FormControl,
  FormControlLabel,
  FormGroup,
  IconButton,
  MenuItem,
  TextField,
  useMediaQuery,
} from '@mui/material';
import { Level } from 'api-client';
import { ChangeEvent } from 'react';
import React from 'react';

import { AppEvents } from '../app-events';

interface LayersControllerProps {
  disabledLayers: Record<string, boolean>;
  onChange: (event: ChangeEvent<HTMLInputElement>, value: string) => void;
  levels: Level[];
  currentLevel: Level;
  handleFullView: () => void;
  handleZoomIn: () => void;
  handleZoomOut: () => void;
}

export const LayersController = ({
  disabledLayers,
  onChange,
  levels,
  currentLevel,
  handleFullView,
  handleZoomIn,
  handleZoomOut,
}: LayersControllerProps) => {
  const [isHovered, setIsHovered] = React.useState(false);
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');

  return (
    <Box
      component="div"
      sx={{
        position: 'absolute',
        top: '60px',
        left: '4px',
        width: 'auto',
        height: 'auto',
        zIndex: '1',
      }}
    >
      <FormControl>
        <TextField
          select
          id="level-select"
          label="Levels"
          variant="outlined"
          fullWidth
          margin="none"
          value={currentLevel.name}
          size="small"
          sx={{ width: '80px' }}
          onChange={(e: ChangeEvent<HTMLInputElement>) => onChange(e, e.target.value as string)}
        >
          {levels.map((level, i) => (
            <MenuItem key={i} value={level.name}>
              {level.name}
            </MenuItem>
          ))}
        </TextField>
      </FormControl>
      <div>
        <IconButton
          size="small"
          onClick={handleFullView}
          data-testid="full-view"
          sx={{ padding: isScreenHeightLessThan800 ? 0 : 2 }}
        >
          <FullscreenIcon
            fontSize="large"
            transform={`scale(${isScreenHeightLessThan800 ? 0.5 : 1})`}
          />
        </IconButton>
      </div>
      <div>
        <IconButton
          size="small"
          onClick={handleZoomIn}
          data-testid="zoom-in"
          sx={{ padding: isScreenHeightLessThan800 ? 0 : 2 }}
        >
          <ZoomInIcon
            fontSize="large"
            transform={`scale(${isScreenHeightLessThan800 ? 0.5 : 1})`}
          />
        </IconButton>
      </div>
      <div>
        <IconButton
          size="small"
          onClick={handleZoomOut}
          data-testid="zoom-out"
          sx={{ padding: isScreenHeightLessThan800 ? 0 : 2 }}
        >
          <ZoomOutIcon
            fontSize="large"
            transform={`scale(${isScreenHeightLessThan800 ? 0.5 : 1})`}
          />
        </IconButton>
      </div>
      <div onMouseEnter={() => setIsHovered(true)} onMouseLeave={() => setIsHovered(false)}>
        <IconButton
          size="small"
          data-testid="layers"
          sx={{ padding: isScreenHeightLessThan800 ? 0 : 2 }}
        >
          <LayersIcon
            fontSize="large"
            transform={`scale(${isScreenHeightLessThan800 ? 0.5 : 1})`}
          />
        </IconButton>
        {isHovered && (
          <div>
            {Object.keys(disabledLayers).map((layerName) => (
              <FormGroup key={layerName}>
                <FormControlLabel
                  control={
                    <Checkbox
                      sx={{ padding: isScreenHeightLessThan800 ? 0 : 1.5 }}
                      size="small"
                      checked={!disabledLayers[layerName]}
                      onChange={() => {
                        const updatedLayers = { ...disabledLayers };
                        updatedLayers[layerName] = !updatedLayers[layerName];
                        AppEvents.disabledLayers.next(updatedLayers);
                      }}
                    />
                  }
                  label={
                    isScreenHeightLessThan800 ? (
                      <span
                        style={{
                          fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
                          marginLeft: isScreenHeightLessThan800 ? '4px' : '0px',
                        }}
                      >
                        {layerName}
                      </span>
                    ) : (
                      layerName
                    )
                  }
                  sx={{ margin: '0' }}
                />
              </FormGroup>
            ))}
          </div>
        )}
      </div>
    </Box>
  );
};
