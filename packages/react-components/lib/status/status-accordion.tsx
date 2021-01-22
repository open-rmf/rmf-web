import React from 'react';
import { Accordion, AccordionDetails, makeStyles } from '@material-ui/core';
import ItemAccordionSummary from '../item-accordion-summary';
import { StatusCard, StatusIndicator } from './index';

export interface StatusAccordionProps {
  statusIndicators: StatusIndicator;
}

const useStyles = makeStyles((theme) => ({
  serviceIndicator: {
    height: '1.5rem',
    width: '1.5rem',
    borderRadius: '50%',
    border: 'none',
  },
  online: {
    backgroundColor: theme.palette.success.main,
  },
  error: {
    backgroundColor: theme.palette.error.main,
  },
}));

export const StatusAccordion = React.forwardRef(
  (props: StatusAccordionProps, ref: React.Ref<HTMLElement>) => {
    const classes = useStyles();
    const { statusIndicators } = props;
    const [itemIndicator, setItemIndicator] = React.useState<{ [key: string]: boolean }>({});
    const [serviceIndicatorStyle, setServiceIndicatorStyle] = React.useState(classes.online);

    React.useEffect(() => {
      const initialItemIndicator: { [key: string]: boolean } = {
        doors: true,
        lifts: true,
        robots: true,
        dispensers: true,
      };
      Object.keys(statusIndicators).forEach((category) => {
        Object.keys(statusIndicators[category]).forEach((item) => {
          if (!statusIndicators[category][item].state) {
            setServiceIndicatorStyle(classes.error);
            initialItemIndicator[category] = false;
            setItemIndicator(initialItemIndicator);
          }
        });
      });
    }, [statusIndicators, classes.error]);

    return (
      <Accordion ref={ref}>
        <ItemAccordionSummary
          title={'All services are online'}
          statusProps={{
            className: `${classes.serviceIndicator} ${serviceIndicatorStyle}`,
            text: '',
            variant: 'normal',
          }}
        />
        <AccordionDetails>
          <StatusCard statusIndicators={statusIndicators} itemIndicator={itemIndicator} />
        </AccordionDetails>
      </Accordion>
    );
  },
);
