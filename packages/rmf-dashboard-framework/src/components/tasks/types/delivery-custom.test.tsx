import { fireEvent, render, screen, within } from '@testing-library/react';
import { describe, expect, it, vi } from 'vitest';

import {
  DeliveryAreaPickupTaskDefinition,
  deliveryCustomInsertCartId,
  deliveryCustomInsertDropoff,
  deliveryCustomInsertOnCancel,
  deliveryCustomInsertPickup,
  DeliveryCustomTaskDescription,
  DeliveryCustomTaskForm,
  deliveryInsertCartId,
  deliveryInsertDropoff,
  deliveryInsertOnCancel,
  deliveryInsertPickup,
  DeliveryPickupTaskDefinition,
  DeliveryPickupTaskDescription,
  DeliveryPickupTaskForm,
  DeliverySequentialLotPickupTaskDefinition,
  isDeliveryCustomTaskDescriptionValid,
  isDeliveryPickupTaskDescriptionValid,
  makeDefaultDeliveryCustomTaskDescription,
  makeDefaultDeliveryPickupTaskDescription,
  makeDeliveryCustomTaskBookingLabel,
  makeDeliveryCustomTaskShortDescription,
  makeDeliveryPickupTaskBookingLabel,
  makeDeliveryPickupTaskShortDescription,
} from '.';

const mockPickupPoints = {
  pickup_1: 'handler_1',
  pickup_2: 'handler_2',
};
const mockCartIds = ['cart_1', 'cart_2', 'cart_3'];
const mockDropoffPoints = {
  dropoff_1: 'handler_3',
  dropoff_2: 'handler_4',
};
const mockPickupZones = {
  pickup_1: 'zone_1',
  pickup_2: 'zone_2',
};

describe('Custom deliveries', () => {
  it('delivery pickup', () => {
    let deliveryPickupTaskDescription: DeliveryPickupTaskDescription | null = null;
    try {
      deliveryPickupTaskDescription = JSON.parse(`{
        "category": "delivery_pickup",
        "phases": [
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "go_to_place",
                    "description": "test_pickup_place"
                  },
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_pickup",
                      "description": {
                        "cart_id": "test_cart_id",
                        "pickup_lot": "test_pickup_lot"
                      }
                    }
                  }
                ]
              }
            }
          },
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "go_to_place",
                    "description": "test_dropoff_place"
                  }
                ]
              }
            },
            "on_cancel": [
              {
                "category": "sequence",
                "description": [
                  {
                    "category": "go_to_place",
                    "description": {
                      "one_of": [
                        {
                          "waypoint": "test_waypoint_1"
                        },
                        {
                          "waypoint": "test_waypoint_2"
                        },
                        {
                          "waypoint": "test_waypoint_3"
                        }
                      ],
                      "constraints": [
                        {
                          "category": "prefer_same_map",
                          "description": ""
                        }
                      ]
                    }
                  },
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_dropoff",
                      "description": {}
                    }
                  }
                ]
              }
            ]
          },
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_dropoff",
                      "description": {}
                    }
                  }
                ]
              }
            }
          }
        ]
      }
      `) as DeliveryPickupTaskDescription;
    } catch (_e) {
      deliveryPickupTaskDescription = null;
    }
    expect(deliveryPickupTaskDescription).not.toEqual(null);

    let description = makeDefaultDeliveryPickupTaskDescription();
    description = deliveryInsertPickup(description, 'test_pickup_place', 'test_pickup_lot');
    description = deliveryInsertCartId(description, 'test_cart_id');
    description = deliveryInsertDropoff(description, 'test_dropoff_place');
    description = deliveryInsertOnCancel(description, [
      'test_waypoint_1',
      'test_waypoint_2',
      'test_waypoint_3',
    ]);
    expect(deliveryPickupTaskDescription).toEqual(description);
  });

  it('delivery pickup task form renders, changes and validates', async () => {
    const onChange = vi.fn();
    const onValidate = vi.fn();

    render(
      <DeliveryPickupTaskForm
        taskDesc={makeDefaultDeliveryPickupTaskDescription()}
        pickupPoints={mockPickupPoints}
        cartIds={mockCartIds}
        dropoffPoints={mockDropoffPoints}
        onChange={onChange}
        onValidate={onValidate}
      />,
    );

    let triggerCount = 1;

    const pickupPlace = screen.getByTestId('pickup-location');
    let input = within(pickupPlace).getByLabelText(/pickup location/i);
    pickupPlace.focus();
    fireEvent.change(input, { target: { value: 'a' } });
    fireEvent.keyDown(pickupPlace, { key: 'ArrowDown' });
    fireEvent.keyDown(pickupPlace, { key: 'Enter' });
    expect(onChange).toHaveBeenCalledTimes(triggerCount);
    expect(onValidate).toHaveBeenCalledTimes(triggerCount);
    triggerCount += 1;

    const cartId = screen.getByTestId('cart-id');
    cartId.focus();
    input = within(cartId).getByLabelText(/cart id/i);
    fireEvent.change(input, { target: { value: 'a' } });
    fireEvent.keyDown(pickupPlace, { key: 'ArrowDown' });
    fireEvent.keyDown(pickupPlace, { key: 'Enter' });
    expect(onChange).toHaveBeenCalledTimes(triggerCount);
    expect(onValidate).toHaveBeenCalledTimes(triggerCount);
    triggerCount += 1;

    const dropoffPlace = screen.getByTestId('dropoff-location');
    dropoffPlace.focus();
    input = within(dropoffPlace).getByLabelText(/dropoff location/i);
    fireEvent.change(input, { target: { value: 'a' } });
    fireEvent.keyDown(dropoffPlace, { key: 'ArrowDown' });
    fireEvent.keyDown(dropoffPlace, { key: 'Enter' });
    expect(onChange).toHaveBeenCalledTimes(triggerCount);
    expect(onValidate).toHaveBeenCalledTimes(triggerCount);
    triggerCount += 1;
  });

  it('delivery pickup booking label', () => {
    let desc = makeDefaultDeliveryPickupTaskDescription();
    desc = deliveryInsertPickup(desc, 'test_place', 'test_lot');
    desc = deliveryInsertCartId(desc, 'test_cart_id');
    desc = deliveryInsertDropoff(desc, 'test_dropoff');
    const label = makeDeliveryPickupTaskBookingLabel(desc);
    expect(label.task_definition_id).toBe(DeliveryPickupTaskDefinition.taskDefinitionId);
    expect(label.pickup).toBe('test_lot');
    expect(label.destination).toBe('test_dropoff');
    expect(label.cart_id).toBe('test_cart_id');
  });

  it('delivery pickup validity', () => {
    let desc = makeDefaultDeliveryPickupTaskDescription();
    expect(
      isDeliveryPickupTaskDescriptionValid(desc, mockPickupPoints, mockDropoffPoints),
    ).not.toBeTruthy();
    desc = deliveryInsertPickup(desc, 'pickup_1', 'handler_1');
    desc = deliveryInsertCartId(desc, 'cart_1');
    desc = deliveryInsertDropoff(desc, 'dropoff_1');
    expect(
      isDeliveryPickupTaskDescriptionValid(desc, mockPickupPoints, mockDropoffPoints),
    ).toBeTruthy();
  });

  it('delivery pickup short description', () => {
    let desc = makeDefaultDeliveryPickupTaskDescription();
    desc = deliveryInsertPickup(desc, 'pickup_1', 'handler_1');
    desc = deliveryInsertCartId(desc, 'cart_1');
    desc = deliveryInsertDropoff(desc, 'dropoff_1');
    expect(makeDeliveryPickupTaskShortDescription(desc, undefined)).toBe(
      '[Delivery - 1:1] payload [cart_1] from [pickup_1] to [dropoff_1]',
    );
  });

  it('delivery_sequential_lot_pickup', () => {
    let deliveryCustomTaskDescription: DeliveryCustomTaskDescription | null = null;
    try {
      deliveryCustomTaskDescription = JSON.parse(`{
        "category": "delivery_sequential_lot_pickup",
        "phases": [
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "go_to_place",
                    "description": "test_pickup_place"
                  },
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_sequential_lot_pickup",
                      "description": {
                        "cart_id": "test_cart_id",
                        "pickup_zone": "test_pickup_zone"
                      }
                    }
                  }
                ]
              }
            }
          },
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "go_to_place",
                    "description": "test_dropoff_place"
                  }
                ]
              }
            },
            "on_cancel": [
              {
                "category": "sequence",
                "description": [
                  {
                    "category": "go_to_place",
                    "description": {
                      "one_of": [
                        {
                          "waypoint": "test_waypoint_1"
                        },
                        {
                          "waypoint": "test_waypoint_2"
                        },
                        {
                          "waypoint": "test_waypoint_3"
                        }
                      ],
                      "constraints": [
                        {
                          "category": "prefer_same_map",
                          "description": ""
                        }
                      ]
                    }
                  },
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_dropoff",
                      "description": {}
                    }
                  }
                ]
              }
            ]
          },
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_dropoff",
                      "description": {}
                    }
                  }
                ]
              }
            }
          }
        ]
      }
      `) as DeliveryCustomTaskDescription;
    } catch (_e) {
      deliveryCustomTaskDescription = null;
    }
    expect(deliveryCustomTaskDescription).not.toEqual(null);

    let description: DeliveryCustomTaskDescription = makeDefaultDeliveryCustomTaskDescription(
      'delivery_sequential_lot_pickup',
    );
    description = deliveryCustomInsertPickup(description, 'test_pickup_place', 'test_pickup_zone');
    description = deliveryCustomInsertCartId(description, 'test_cart_id');
    description = deliveryCustomInsertDropoff(description, 'test_dropoff_place');
    description = deliveryCustomInsertOnCancel(description, [
      'test_waypoint_1',
      'test_waypoint_2',
      'test_waypoint_3',
    ]);
    expect(deliveryCustomTaskDescription).toEqual(description);
  });

  it('delivery_area_pickup', () => {
    let deliveryCustomTaskDescription: DeliveryCustomTaskDescription | null = null;
    try {
      deliveryCustomTaskDescription = JSON.parse(`{
        "category": "delivery_area_pickup",
        "phases": [
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "go_to_place",
                    "description": "test_pickup_place"
                  },
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_area_pickup",
                      "description": {
                        "cart_id": "test_cart_id",
                        "pickup_zone": "test_pickup_zone"
                      }
                    }
                  }
                ]
              }
            }
          },
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "go_to_place",
                    "description": "test_dropoff_place"
                  }
                ]
              }
            },
            "on_cancel": [
              {
                "category": "sequence",
                "description": [
                  {
                    "category": "go_to_place",
                    "description": {
                      "one_of": [
                        {
                          "waypoint": "test_waypoint_1"
                        },
                        {
                          "waypoint": "test_waypoint_2"
                        },
                        {
                          "waypoint": "test_waypoint_3"
                        }
                      ],
                      "constraints": [
                        {
                          "category": "prefer_same_map",
                          "description": ""
                        }
                      ]
                    }
                  },
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_dropoff",
                      "description": {}
                    }
                  }
                ]
              }
            ]
          },
          {
            "activity": {
              "category": "sequence",
              "description": {
                "activities": [
                  {
                    "category": "perform_action",
                    "description": {
                      "unix_millis_action_duration_estimate": 60000,
                      "category": "delivery_dropoff",
                      "description": {}
                    }
                  }
                ]
              }
            }
          }
        ]
      }
      `) as DeliveryCustomTaskDescription;
    } catch (_e) {
      deliveryCustomTaskDescription = null;
    }
    expect(deliveryCustomTaskDescription).not.toEqual(null);

    let description: DeliveryCustomTaskDescription =
      makeDefaultDeliveryCustomTaskDescription('delivery_area_pickup');
    description = deliveryCustomInsertPickup(description, 'test_pickup_place', 'test_pickup_zone');
    description = deliveryCustomInsertCartId(description, 'test_cart_id');
    description = deliveryCustomInsertDropoff(description, 'test_dropoff_place');
    description = deliveryCustomInsertOnCancel(description, [
      'test_waypoint_1',
      'test_waypoint_2',
      'test_waypoint_3',
    ]);
    expect(deliveryCustomTaskDescription).toEqual(description);
  });

  it('delivery custom task form renders, changes and validates', async () => {
    const onChange = vi.fn();
    const onValidate = vi.fn();

    render(
      <DeliveryCustomTaskForm
        taskDesc={makeDefaultDeliveryCustomTaskDescription('delivery_sequential_lot_pickup')}
        pickupZones={Object.keys(mockPickupZones)}
        cartIds={mockCartIds}
        dropoffPoints={Object.keys(mockDropoffPoints)}
        onChange={onChange}
        onValidate={onValidate}
      />,
    );

    let triggerCount = 1;

    const pickupPlace = screen.getByTestId('pickup-zone');
    let input = within(pickupPlace).getByLabelText(/pickup zone/i);
    pickupPlace.focus();
    fireEvent.change(input, { target: { value: 'a' } });
    fireEvent.keyDown(pickupPlace, { key: 'ArrowDown' });
    fireEvent.keyDown(pickupPlace, { key: 'Enter' });
    expect(onChange).toHaveBeenCalledTimes(triggerCount);
    expect(onValidate).toHaveBeenCalledTimes(triggerCount);
    triggerCount += 1;

    const cartId = screen.getByTestId('cart-id');
    cartId.focus();
    input = within(cartId).getByLabelText(/cart id/i);
    fireEvent.change(input, { target: { value: 'a' } });
    fireEvent.keyDown(pickupPlace, { key: 'ArrowDown' });
    fireEvent.keyDown(pickupPlace, { key: 'Enter' });
    expect(onChange).toHaveBeenCalledTimes(triggerCount);
    expect(onValidate).toHaveBeenCalledTimes(triggerCount);
    triggerCount += 1;

    const dropoffPlace = screen.getByTestId('dropoff-location');
    dropoffPlace.focus();
    input = within(dropoffPlace).getByLabelText(/dropoff location/i);
    fireEvent.change(input, { target: { value: 'a' } });
    fireEvent.keyDown(dropoffPlace, { key: 'ArrowDown' });
    fireEvent.keyDown(dropoffPlace, { key: 'Enter' });
    expect(onChange).toHaveBeenCalledTimes(triggerCount);
    expect(onValidate).toHaveBeenCalledTimes(triggerCount);
    triggerCount += 1;
  });

  it('delivery custom booking label', () => {
    let desc = makeDefaultDeliveryCustomTaskDescription('delivery_sequential_lot_pickup');
    desc = deliveryCustomInsertPickup(desc, 'test_place', 'test_lot');
    desc = deliveryCustomInsertCartId(desc, 'test_cart_id');
    desc = deliveryCustomInsertDropoff(desc, 'test_dropoff');
    let label = makeDeliveryCustomTaskBookingLabel(desc);
    expect(label.task_definition_id).toBe(
      DeliverySequentialLotPickupTaskDefinition.taskDefinitionId,
    );
    expect(label.pickup).toBe('test_lot');
    expect(label.destination).toBe('test_dropoff');
    expect(label.cart_id).toBe('test_cart_id');

    desc = makeDefaultDeliveryCustomTaskDescription('delivery_area_pickup');
    desc = deliveryCustomInsertPickup(desc, 'test_place', 'test_lot');
    desc = deliveryCustomInsertCartId(desc, 'test_cart_id');
    desc = deliveryCustomInsertDropoff(desc, 'test_dropoff');
    label = makeDeliveryCustomTaskBookingLabel(desc);
    expect(label.task_definition_id).toBe(DeliveryAreaPickupTaskDefinition.taskDefinitionId);
    expect(label.pickup).toBe('test_lot');
    expect(label.destination).toBe('test_dropoff');
    expect(label.cart_id).toBe('test_cart_id');
  });

  it('delivery custom validity', () => {
    let desc = makeDefaultDeliveryCustomTaskDescription('delivery_sequential_lot_pickup');
    expect(
      isDeliveryCustomTaskDescriptionValid(
        desc,
        Object.values(mockPickupZones),
        Object.keys(mockDropoffPoints),
      ),
    ).not.toBeTruthy();
    desc = deliveryCustomInsertPickup(desc, 'pickup_1', 'zone_1');
    desc = deliveryCustomInsertCartId(desc, 'cart_1');
    desc = deliveryCustomInsertDropoff(desc, 'dropoff_1');
    expect(
      isDeliveryCustomTaskDescriptionValid(
        desc,
        Object.values(mockPickupZones),
        Object.keys(mockDropoffPoints),
      ),
    ).toBeTruthy();
  });

  it('delivery custom short description', () => {
    let desc = makeDefaultDeliveryCustomTaskDescription('delivery_sequential_lot_pickup');
    desc = deliveryCustomInsertPickup(desc, 'pickup_1', 'zone_1');
    desc = deliveryCustomInsertCartId(desc, 'cart_1');
    desc = deliveryCustomInsertDropoff(desc, 'dropoff_1');
    expect(makeDeliveryCustomTaskShortDescription(desc, undefined)).toBe(
      '[Delivery - Sequential lot pick up] payload [cart_1] from [pickup_1] to [dropoff_1]',
    );
  });
});
