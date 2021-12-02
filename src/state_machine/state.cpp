// Copyright (C) 2021 Alessandro Fornasier,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <alessandro.fornasier@ieee.org>
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

#include "state_machine/states/undefined.h"
#include "state_machine/states/nominal.h"
#include "state_machine/states/hold.h"
#include "state_machine/states/failure.h"
#include "state_machine/states/initialization.h"
#include "state_machine/states/land.h"
#include "state_machine/states/preflight.h"
#include "state_machine/states/start_mission.h"
#include "state_machine/states/perform_mission.h"
#include "state_machine/states/end_mission.h"
#include "state_machine/states/termination.h"

namespace autonomy {

  State::~State() {}

} // namespace autonomy
