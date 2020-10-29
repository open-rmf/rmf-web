import { AccordionProps } from '@material-ui/core';
import React from 'react';

type ManagedProps = 'onChange' | 'expanded';

export interface SpotlightHandle {
  spotlight(): void;
}

export type SpotlightAccordionProps<P extends OptionalChildren<AccordionProps>> = Omit<
  P,
  ManagedProps
>;

type OptionalChildren<T extends React.PropsWithChildren<unknown>> = Omit<T, 'children'> & {
  children?: T['children'];
};

/**
 * Given an Accordion component, add support for putting it on "spotlight". This customize the
 * component so that it returns a ref with a `spotlight` method. Calling that method will scroll
 * the component into view and expand it.
 *
 * This overrides the `onChange` and `expanded` props so those are removed from the public interface.
 * @param BaseAccordion
 */
export function makeSpotlightAccordion<P extends OptionalChildren<AccordionProps>>(
  BaseAccordion: React.ComponentType<P>,
): React.ForwardRefExoticComponent<
  React.PropsWithoutRef<SpotlightAccordionProps<P>> & React.RefAttributes<SpotlightHandle>
> {
  return React.forwardRef((props: SpotlightAccordionProps<P>, ref: React.Ref<SpotlightHandle>) => {
    const [expanded, setExpanded] = React.useState(false);
    const innerRef = React.useRef<HTMLElement>();

    React.useImperativeHandle(ref, () => {
      return {
        spotlight: () => {
          setExpanded(true);
          innerRef.current?.scrollIntoView({ behavior: 'smooth' });
        },
      };
    });

    return (
      <BaseAccordion ref={ref} expanded={expanded} {...(props as P)}>
        {props.children}
      </BaseAccordion>
    );
  });
}
