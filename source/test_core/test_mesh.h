/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, Sergey Ushakov
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <kinverse/core/mesh.h>

namespace kinverse {
  namespace core {

    using MeshEquality = std::tuple<Mesh, Mesh, bool>;

    class TestMesh : public testing::Test, public testing::WithParamInterface<MeshEquality> {};

    INSTANTIATE_TEST_SUITE_P(
        EqualityOperator,
        TestMesh,
        testing::Values(

            // case of equal meshes
            MeshEquality{ Mesh{ Mesh::Vertices{ { 1.0, 1.0, 1.0 }, { 2.0, 3.0, 4.0 } }, Mesh::Faces{ { 0, 1, 2 }, { 3, 4, 5, 6 } } },
                          Mesh{ Mesh::Vertices{ { 1.0, 1.0, 1.0 }, { 2.0, 3.0, 4.0 } }, Mesh::Faces{ { 0, 1, 2 }, { 3, 4, 5, 6 } } },
                          true },

            // case of different index buffers
            MeshEquality{ Mesh{ Mesh::Vertices{ { 1.0, 1.0, 1.0 }, { 2.0, 3.0, 4.0 } }, Mesh::Faces{ { 0, 1, 2 }, { 3, 4, 5, 6 } } },
                          Mesh{ Mesh::Vertices{ { 1.0, 1.0, 1.0 }, { 2.0, 3.0, 4.0 } }, Mesh::Faces{ { 0, 1, 2 }, { 3, 4, 5, 6 }, {} } },
                          false },
            MeshEquality{ Mesh{ Mesh::Vertices{ { 1.0, 1.0, 1.0 }, { 2.0, 3.0, 4.0 } }, Mesh::Faces{ { 0, 1, 2 }, { 3, 4, 5, 6 } } },
                          Mesh{ Mesh::Vertices{ { 1.0, 1.0, 1.0 }, { 2.0, 3.0, 4.0 } }, Mesh::Faces{ { 0, 1, 2 }, { 3, 4, 5, 6 }, { 1 } } },
                          false },
            MeshEquality{ Mesh{ Mesh::Vertices{ { 1.0, 1.0, 1.0 }, { 2.0, 3.0, 4.0 } }, Mesh::Faces{ { 0, 1, 2 }, { 3, 4, 5, 6 } } },
                          Mesh{ Mesh::Vertices{ { 1.0, 1.0, 1.0 }, { 2.0, 3.0, 4.0 } }, Mesh::Faces{ { 0, 1, 2 }, { 3, 4, 5, 7 } } },
                          false },
            MeshEquality{ Mesh{ Mesh::Vertices{ { 1.0, 1.0, 1.0 }, { 2.0, 3.0, 4.0 } }, Mesh::Faces{ { 0, 1, 2 }, { 3, 4, 5, 6 } } },
                          Mesh{ Mesh::Vertices{ { 1.0, 1.0, 1.0 }, { 2.0, 3.0, 4.0 } }, Mesh::Faces{ { 0, 1, 2 }, { 3, 4, 5 }, {} } },
                          false },

            // case of different vertex buffers
            MeshEquality{ Mesh{ Mesh::Vertices{ { 1.0, 1.0, 1.0 }, { 2.0, 3.0, 4.0 } }, Mesh::Faces{ { 0, 1, 2 }, { 3, 4, 5, 6 } } },
                          Mesh{ Mesh::Vertices{ { 1.0, 1.0, 1.0 }, { 2.0, 3.0, 4.0 }, { 5.0, 5.0, 5.0 } }, Mesh::Faces{ { 0, 1, 2 }, { 3, 4, 5, 6 } } },
                          false },
            MeshEquality{ Mesh{ Mesh::Vertices{ { 1.0, 1.0, 1.0 }, { 2.0, 3.0, 4.0 } }, Mesh::Faces{ { 0, 1, 2 }, { 3, 4, 5, 6 } } },
                          Mesh{ Mesh::Vertices{ { 1.0, 1.0, 1.0 }, { 4.0, 5.0, 6.0 } }, Mesh::Faces{ { 0, 1, 2 }, { 3, 4, 5, 6 } } },
                          false },
            MeshEquality{
                Mesh{ Mesh::Vertices{ { 0.000000000001, 0.0, 0.0 } }, Mesh::Faces{} }, Mesh{ Mesh::Vertices{ { 0.0, 0.0, 0.0 } }, Mesh::Faces{} }, false }

            ));

  }  // namespace core
}  // namespace kinverse
