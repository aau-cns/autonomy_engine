// Copyright (C) 2021 Christian Brommer and Alessandro Fornasier,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <christian.brommer@ieee.org>
// and <alessandro.fornasier@ieee.org>
//
// All rights reserved.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#include "autonomy_core/entity_event.h"

EntityEvent::EntityEvent() {}

const Entity& EntityEvent::getEntity() const {
  return entity_;
}

const Type& EntityEvent::getType() const {
  return type_;
}

const subType& EntityEvent::getSubType() const {
  return sub_type_;
}

const NextState& EntityEvent::getNextState() const {
  return next_state_;
}

const bool& EntityEvent::is_init() const {
  return init_;
}

void EntityEvent::setEvent(const Entity& entity, const Type& type, const subType& subtype, const NextState& next_state) {
  entity_ = entity;
  type_ = type;
  sub_type_ = subtype;
  next_state_ = next_state;
  init_ = true;
}
