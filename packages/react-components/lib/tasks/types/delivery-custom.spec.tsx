import {
  deliveryCustomInsertCartId,
  deliveryCustomInsertDropoff,
  deliveryCustomInsertOnCancel,
  deliveryCustomInsertPickup,
  DeliveryCustomTaskDescription,
  deliveryInsertCartId,
  deliveryInsertDropoff,
  deliveryInsertOnCancel,
  deliveryInsertPickup,
  DeliveryPickupTaskDescription,
  makeDefaultDeliveryCustomTaskDescription,
  makeDefaultDeliveryPickupTaskDescription,
} from '.';

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
    } catch (e) {
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
    } catch (e) {
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
    } catch (e) {
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
});
